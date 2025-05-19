#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <map>
#include <string>
#include <vector>

// Target joint to move
const std::string TARGET_JOINT = "fr3_joint7";  // Move the joint7 -> rotate the end effector
const double TARGET_POSITION = 1.57;  // Radians 90 deg
const double TRAJECTORY_DURATION = 2.0;  // Seconds

// The joint order expected by the controller
const std::vector<std::string> CONTROLLER_JOINTS = {
    "fr3_joint1", "fr3_joint2", "fr3_joint3",
    "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
};

// Global storage of current joint positions
std::map<std::string, double> joint_positions;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        joint_positions[msg->name[i]] = msg->position[i];
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_mover");
    ros::NodeHandle nh;

    // Subscribe to /joint_states to get current positions
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);

    // Wait until we receive joint positions
    ros::Rate rate(10);
    ROS_INFO("Waiting for joint_states...");
    while (ros::ok()) {
        ros::spinOnce();
        bool all_found = true;
        for (const auto& joint : CONTROLLER_JOINTS) {
            if (joint_positions.find(joint) == joint_positions.end()) {
                all_found = false;
                break;
            }
        }
        if (all_found) break;
        rate.sleep();
    }


    // Define the action client (true = spin thread)
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client(
        "/fr3/arm_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for action server to start...");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // Create a goal to send
    control_msgs::FollowJointTrajectoryGoal goal;

    // Specify the joint names you want to move
    goal.trajectory.joint_names = CONTROLLER_JOINTS; 

    trajectory_msgs::JointTrajectoryPoint point;
    // Define a single point in the trajectory
    for (const auto& joint : CONTROLLER_JOINTS) {
        if (joint == TARGET_JOINT) {
            point.positions.push_back(TARGET_POSITION);
        } else {
            point.positions.push_back(joint_positions[joint]);
        }
    }

    point.time_from_start = ros::Duration(TRAJECTORY_DURATION); // Time to reach the target

    // Add the point to the trajectory
    goal.trajectory.points.push_back(point);

    // Set the header time stamp
    goal.trajectory.header.stamp = ros::Time::now();

    // Send the goal
    client.sendGoal(goal);

    // Send and wait
    ROS_INFO("Sending trajectory to move %s", TARGET_JOINT.c_str());
    client.sendGoal(goal);
    client.waitForResult();
    ROS_INFO("Movement complete.");

    return 0;
}
