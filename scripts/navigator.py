#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf

class Navigator:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.status_sub = rospy.Subscriber("move_base/result", MoveBaseActionResult, self.callback)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.tf_listener = tf.TransformListener()

    def get_robot_position(self):
        try:
            self.tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            return trans,rot
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to get robot's position.")
            return None

    def send_goal_through_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id='map'
        goal.target_pose.pose.position.x = 0.5899236798286438
        goal.target_pose.pose.position.y = 1.3233416080474854
        goal.target_pose.pose.orientation.z = 0.8596124084983308
        goal.target_pose.pose.orientation.w = 0.5109466774093936  
        self.client.send_goal(goal)
        wait = self.client.wait_for_result(rospy.Duration.from_sec(30.0))
        if not wait:
            str_log="The Goal Planning Failed for some reasons"
            rospy.loginfo(str_log)
        else:
            str_log="The Goal achieved success"
            rospy.loginfo(str_log)


    def send_goals(self):
        goal = PoseStamped()
        goal.header.frame_id='map'
        goal.pose.position.x = 0.5899236798286438
        goal.pose.position.y = 1.3233416080474854
        goal.pose.orientation.z = 0.8596124084983308
        goal.pose.orientation.w = 0.5109466774093936
        self.goal_pub.publish(goal)

    def callback(self, msg):
        print(msg.status.status)
    

if __name__ == '__main__':
    rospy.init_node('navigator')
    navigator = Navigator()
    navigator.send_goal_through_client()
    navigator.get_robot_position()
    rospy.spin()