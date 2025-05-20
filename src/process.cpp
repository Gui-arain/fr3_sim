#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
//Include joint service
//#include "fr3_sim/MoveSingleJoint.h"
//for keyboard input
#include <iostream>

// tau = 1 rotation in radiants
const double tau = 2 * M_PI;

//Position of the elements
const double hole_spacing = 0.05;
const double table_space = 0.1;
//Well plate postition of the first well 
const double well_plate_x = table_space + 0.3255;
const double well_plate_y = 0.1315;
const double well_plate_z = 0.07;
//tip holder postion of the first hole 0.525, -0.325, 0.05
const double tip_holder_x = table_space + 0.025 + 6*hole_spacing + 0.02; //0.525 + 0.02;
const double tip_holder_y = -0.025 -6*hole_spacing + 0.005; //-0.325 + 0.005;
const double tip_holder_z = 0.09;
//beaker center pos 0.725, 0.025, 0.18
const double beaker_x = table_space + 0.025 + 14*hole_spacing; // 0.725;
const double beaker_y = hole_spacing/2;
const double beaker_z = 0.2;
//tip ejector 1000uL center pos
const double tip_ejector_x = table_space + 0.025 + 15*hole_spacing - 0.045 - 0.04;
const double tip_ejector_y = -0.025 - 7*hole_spacing + 0.206;
const double tip_ejector_z = 0.155065;

void waitForEnter()
{
    std::cout << "Press Enter to continue to the next motion..." << std::endl;
    std::cin.get();
}

void plan_move(moveit::planning_interface::MoveGroupInterface& move_group, std::array<double, 3> target_pos)
{
    move_group.setEndEffectorLink("fr3_link_pip_tip");  // Adjust to your robot's EE link
    move_group.setPoseReferenceFrame("world");

    // Desired yaw angle (rotation around Z)
    double theta = atan2(target_pos[1], target_pos[0]) - M_PI/2;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);  // Roll = 0, Pitch = 0, Yaw = theta

    geometry_msgs::Pose target_pose_msg;
    target_pose_msg.orientation = tf2::toMsg(q);  // Automatically normalized
    target_pose_msg.position.x = target_pos[0]; //0.625;
    target_pose_msg.position.y = target_pos[1]; //0.025;
    target_pose_msg.position.z = target_pos[2]; //0.2;

    if (!move_group.setPoseTarget(target_pose_msg)) {
        ROS_WARN("Pose target is invalid or not accepted.");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
}

void plan_pip_rotate(moveit::planning_interface::MoveGroupInterface& move_group, double angle)
{
    move_group.setEndEffectorLink("fr3_link_pip");  // Adjust to your robot's EE link
    move_group.setPoseReferenceFrame("world");

}

void execute_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.setGoalJointTolerance(0.001);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);

    bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Execution %s", success ? "SUCCESS" : "FAILED");
    
}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    //Add the table
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "fr3_link0";

    //Dimensions of the table
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.8;
    collision_objects[0].primitives[0].dimensions[1] = 0.8;
    collision_objects[0].primitives[0].dimensions[2] = 0.02;

    //Pose of the table
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = table_space + 0.4;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.01;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    //Add the table to the scene
    collision_objects[0].operation = collision_objects[0].ADD;

    //Add the pipette tip holder
    collision_objects[1].id = "tip_holder";
    collision_objects[1].header.frame_id = "fr3_link0";
    //Dimensions of tip holder
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.130;
    collision_objects[1].primitives[0].dimensions[1] = 0.08;
    collision_objects[1].primitives[0].dimensions[2] = 0.028;
    //Pose of the tip holder
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = tip_holder_x + 0.03;
    collision_objects[1].primitive_poses[0].position.y = tip_holder_y - 0.03;
    collision_objects[1].primitive_poses[0].position.z = 0.02 + 0.028/2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;

    //Add the tip_holder to the scene
    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "process_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //ros::ServiceClient move_joint_client = nh.serviceClient<fr3_sim::MoveSingleJoint>("move_single_joint");

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface group("fr3_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    group.setPlanningTime(45.0);

    //add the collisions objects in the scene
    addCollisionObject(planning_scene_interface);

    //rotate the end effector
    /*
    fr3_sim::MoveSingleJoint joint_srv;
    joint_srv.request.joint_name = "fr3_joint7";
    joint_srv.request.target_position = 1.57; //Rotate 90 deg
    joint_srv.request.duration = 2.0;
    //call the move_single_joint service
    */
    //move_joint_client.call(joint_srv);

    ros::WallDuration(1.0).sleep();

    // Go to pipette tip box
    plan_move(group, {tip_holder_x, tip_holder_y, tip_holder_z});//{0.525, -0.325, 0.05});

    waitForEnter();  // Pause for keyboard input

    //exexcute the planned path
    execute_move(group);

    ros::WallDuration(1.0).sleep();

    //Move to beaker
    plan_move(group, {beaker_x, beaker_y, beaker_z});

    waitForEnter();  // Pause for keyboard input

    //exexcute the planned path
    execute_move(group);

    ros::WallDuration(1.0).sleep();

    //Move to well plate
    plan_move(group, {well_plate_x, well_plate_y, well_plate_z});

    waitForEnter();  // Pause for keyboard input

    //exexcute the planned path
    execute_move(group);

    ros::WallDuration(1.0).sleep();

    //Move to tip ejector
    plan_move(group, {tip_ejector_x, tip_ejector_y, tip_ejector_z});

    waitForEnter();  // Pause for keyboard input

    //exexcute the planned path
    execute_move(group);

    ros::shutdown();
    return 0;

}