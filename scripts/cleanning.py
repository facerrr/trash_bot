#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from cv_bridge import CvBridge
from copy import deepcopy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import cv2
import os
from arm_controller import Arm

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 0.5
current_path = os.path.dirname(os.path.abspath(__file__))
parent_path = os.path.dirname(current_path)
print(parent_path)

class TrashBot:
    def __init__(self):
        rospy.init_node('trash_bot')
        self.arm = Arm()
        if self.arm.arm_controller:
            self.grabber = self.arm.arm_controller

        self.img_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.imgCallback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.lidarCallback)
        # self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.velCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.depth_pub = rospy.Publisher('/depth', Float32, queue_size=10)
        self.depth_sub = rospy.Subscriber('/depth', Float32, self.depthCallback)
        self.bridge = CvBridge()
        models_path = os.path.join(parent_path, 'train3/weights/best.pt')
        self.model = YOLO(models_path) 

        #camera lidar parameter
        self.camera_offset = (0.076, 0.000, 0.093)
        self.lidar_offset = (-0.024, 0.000, 0.122)
        self.d = [0.1639958233797625, -0.271840030972792, 0.001055841660100477, -0.00166555973740089, 0.0]
        self.k = [322.0704122808738, 0.0, 199.2680620421962, 0.0, 320.8673986158544, 155.2533082600705, 0.0, 0.0, 1.0]

        self.target_class = 'trash'
        #pid controller parameter
        self.enable_pid = False
        self.kp = 0.0015
        self.ki = 0.000005
        self.kd = 0.000
        self.previous_error = 0.0
        self.integral = 0.0
        self.vel = Twist()

        self.rotation = False
        self.isDetected = False
        self.enable_imgCallback = True

        self.img_center_x = 0
        self.box_center_x = 0
        self.depth = 0

        self.neeedAdjust = False
        
    def depthCallback(self, msg):
        if msg.data>0.22:
            self.neeedAdjust = True
        else:
            self.grabber.grab(msg.data)
            print("return the trash successfully")
            self.enable_imgCallback = True
            self.isDetected = False

    def lidarCallback(self,msg):
        ranges = msg.ranges
        self.depth = ranges[0]+self.lidar_offset[0]  # x axis
        if self.neeedAdjust:
            speed_msg = Twist()
            speed_msg.linear.x = 0.1
            self.vel_pub.publish(speed_msg)
            if self.depth<0.22:
                speed_msg = Twist()
                speed_msg.linear.x = 0.0
                self.vel_pub.publish(speed_msg)
                self.depth_pub.publish(self.depth)
                self.neeedAdjust = False
        

    def imgCallback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            if self.enable_imgCallback:
                max_box = None
                max_confidence = 0.0
                result = self.model(cv_image,verbose=False)[0]
                pred_boxes = result.boxes
                img_in = result.orig_img
                font='Arial.ttf'
                names = self.model.names
                annotator = Annotator(
                        im=deepcopy(img_in),
                        font=font,
                        example=names)

                if len(pred_boxes)<1:
                    self.enable_pid = False
                    self.rotation = True
                else:
                    self.enable_pid = True
                    self.rotation = False

                for d in pred_boxes:
                    c, conf, id = int(d.cls), float(d.conf), None if d.id is None else int(d.id.item())
                    name = ('' if id is None else f'id:{id} ') + names[c]
                    label = (f'{name} {conf:.2f}' if conf else name)
                    annotator.box_label(d.xyxy.squeeze(), 'bottle', color=colors(c, True))
                    if conf > max_confidence:
                        max_confidence = conf
                        max_box = d.xyxy.squeeze().tolist()
                if max_box is not None:
                    self.box_center_x = (max_box[0] + max_box[2]) / 2.0
                    self.img_center_x = img_in.shape[1] / 2.0

                if self.rotation:
                        speed_msg = Twist()
                        speed_msg.angular.z = 0.5
                        self.vel_pub.publish(speed_msg)

                else:
                    if self.enable_pid:
                        error = self.img_center_x - self.box_center_x
                        self.integral += error
                        derivative = error - self.previous_error
                        pid_out = self.kp * error + self.ki * self.integral + self.kd * derivative
                        pid_out = self.constrain(pid_out,-WAFFLE_MAX_ANG_VEL,WAFFLE_MAX_ANG_VEL)
                        speed_msg = Twist()
                        speed_msg.angular.z = pid_out
                        # print("control signal:",pid_out)
                        self.vel_pub.publish(speed_msg)

                    if abs(self.img_center_x - self.box_center_x)<3:
                        self.enable_pid = False
                        self.isDetected = True
                        speed_msg = Twist()
                        speed_msg.angular.z = 0
                        self.vel_pub.publish(speed_msg)
                        print("locate the bottle")
                        print("Get the object depth:",self.depth)
                        self.enable_imgCallback = False
                        self.depth_pub.publish(self.depth)
                        

                img_out = annotator.result()
                cv2.imshow("Image", img_out)
                cv2.waitKey(1)

            else:
                cv2.imshow("Image", cv_image)
                cv2.waitKey(1)
                

        except Exception as e:
            rospy.logerr(e)

    def pixel_to_base_link(self,pix,depth):
        point = Point()
        fx = self.k[0]
        fy = self.k[4]
        cx = self.k[2]
        cy = self.k[5]
        x = (pix[0] - cx) / fx
        y = (pix[1] - cy) / fy
        r2 = x*x + y*y
        f = 1 + self.d[0]*r2 + self.d[1]*r2*r2 + self.d[4]*r2*r2*r2
        ux = x*f + 2*self.d[2]*x*y + self.d[3]*(r2 + 2*x*x)
        uy = y*f + 2*self.d[3]*x*y + self.d[2]*(r2 + 2*y*y)

        point.x = depth * ux + self.camera_offset[0]
        point.y = depth * uy + self.camera_offset[1]
        point.z = depth + self.camera_offset[2]
        
        return point

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input
        return input

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            cv2.destroyAllWindows()


# def main():
#     rospy.init_node('trash_detector')
#     detector = Detector()
#     detector.run()
    

if __name__ == '__main__':
    detector = TrashBot()
    detector.run()






