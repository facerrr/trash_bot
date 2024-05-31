#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>
#include <moveit_commander/moveit_commander.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <vector>

class Arm_Contoller
{
    public:
        Arm_Contoller();
        ~Arm_Contoller();

        void setGripper(double arg);
        void moveArm(const std::vector<double>& arg);

    private:
        std::vector<double> poseToList(const geometry_msgs::Pose& pose_msg);
        bool allClose(const std::vector<double>& goal, const std::vector<double>& actual, double tolerance);
        ros::NodeHandle nh_;
        moveit_commander::RobotCommanderPtr robot_;
        moveit_commander::PlanningSceneInterfacePtr scene_;
        moveit_commander::MoveGroupCommanderPtr group_;
        moveit_commander::MoveGroupCommanderPtr grip_;
        ros::Publisher display_trajectory_publisher_;
        actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> execute_trajectory_client_;
        std::string planning_frame_;
        std::string eef_link_;
        std::vector<std::string> group_names_;
        geometry_msgs::PoseStamped home_pose_;
        std::vector<double> home_joint_;
        std::vector<double> home_joint_;
        std::vector<double> place_joint_;
        geometry_msgs::PoseStamped grab_pose_;
}


#endif