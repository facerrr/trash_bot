#include "trash_bot/arm_controller.h"

const double JOINT_TOLERANCE = 0.05;
const double GRIPPER_TOLERANCE = 0.01;
const double POSITION_TOLERANCE = 0.02;

Arm_Contoller::Arm_Contoller()
{
    moveit_commander::initRobot();
    robot_ = moveit_commander::RobotCommanderPtr(new moveit_commander::RobotCommanderPtr());
    scene_ = moveit_commander::PlanningSceneInterfacePtr(new moveit_commander::PlanningSceneInterface());
    std::string group_name = "arm";
    std::string griper_name = "gripper";
    group_ = moveit_commander::MoveGroupCommanderPtr(new moveit_commander::MoveGroupCommander(group_name));
    grip_ = moveit_commander::MoveGroupCommanderPtr(new moveit_commander::MoveGroupCommander(griper_name));
    display_trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    execute_trajectory_client_ = actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>("execute_trajectory", true);
    execute_trajectory_client_.waitForServer();
    group_->setMaxVelocityScalingFactor(1);
    planning_frame_ = group_->getPlanningFrame();
    ROS_INFO_STREAM("Reference frame: " << planning_frame_);
    eef_link_ = group_->getEndEffectorLink();
    ROS_INFO_STREAM("End effector: " << eef_link_);
    group_names_ = robot_->getJointModelGroupNames();
    ROS_INFO_STREAM("Robot Groups:");
    home_pose_ = group_->getCurrentPose();
    home_joint_ = { 0.003067961661145091, -1.6122138500213623, 1.2486604452133179, 0.6550098061561584 };
    hold_joint_ = { 0.0, 0.0, 0.0, 0.0 };
    place_joint_ = { M_PI / 2, 0.0, 0.0, 0.0 };

    grab_pose_ = home_pose_;
    grab_pose_ = home_pose_;
    grab_pose_.pose.position.x = 0.0;
    grab_pose_.pose.position.y = 0.0;
    grab_pose_.pose.position.z = 0.18;
    grab_pose_.pose.orientation.x = 0.0;
    grab_pose_.pose.orientation.y = 0.0;
    grab_pose_.pose.orientation.z = 0.0;
    grab_pose_.pose.orientation.w = 1.0;

}

