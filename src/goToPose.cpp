#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
// tau = 1 rotation in radiants
const double tau = 2 * M_PI;


void plan_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.setEndEffectorLink("fr3_link7");  // Adjust to your robot's EE link
    move_group.setPoseReferenceFrame("world");

    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose();
    ROS_INFO("current orientation x: %f",current_pose.pose.orientation.x);
    ROS_INFO("current orientation y: %f",current_pose.pose.orientation.y);
    ROS_INFO("current orientation z: %f",current_pose.pose.orientation.z);
    ROS_INFO("current orientation w: %f",current_pose.pose.orientation.w);

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = 1.0;
    target_pose1.orientation.y = 0.0;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 0.0;
    target_pose1.position.x = 0.4;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.4;

    if (!move_group.setPoseTarget(target_pose1)) {
        ROS_WARN("Pose target is invalid or not accepted.");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
}


void execute_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.setGoalJointTolerance(0.001);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);

    bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
    
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "goToPose_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface group("fr3_arm");
    group.setPlanningTime(45.0);

    // plan the move
    plan_move(group);

    ros::WallDuration(1.0).sleep();

    //exexcute the planned path
    execute_move(group);

    ros::WallDuration(1.0).sleep();

    ros::waitForShutdown();
    return 0;

}