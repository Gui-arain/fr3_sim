#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <string>
#include <vector>
#include "fr3_sim/MoveSingleJoint.h"  

std::map<std::string, double> joint_positions;

const std::vector<std::string> CONTROLLER_JOINTS = {
    "fr3_joint1", "fr3_joint2", "fr3_joint3",
    "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
};

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        joint_positions[msg->name[i]] = msg->position[i];
    }
}

bool handle_request(
    fr3_sim::MoveSingleJoint::Request &req,
    fr3_sim::MoveSingleJoint::Response &res)
{
    ROS_INFO("Request received");
    if (std::find(CONTROLLER_JOINTS.begin(), CONTROLLER_JOINTS.end(), req.joint_name) == CONTROLLER_JOINTS.end()) {
        res.success = false;
        res.message = "Joint " + req.joint_name + " is not part of the controlled joints.";
        return true;
    }

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client(
        "/fr3/arm_controller/follow_joint_trajectory", true);
    client.waitForServer();

    // Wait for joint_states to populate
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        bool ready = true;
        for (const auto& joint : CONTROLLER_JOINTS) {
            if (joint_positions.find(joint) == joint_positions.end()) {
                ready = false;
                break;
            }
        }
        if (ready) break;
        rate.sleep();
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = CONTROLLER_JOINTS;

    trajectory_msgs::JointTrajectoryPoint point;
    for (const auto& joint : CONTROLLER_JOINTS) {
        if (joint == req.joint_name)
            point.positions.push_back(req.target_position);
        else
            point.positions.push_back(joint_positions[joint]);
    }
    point.time_from_start = ros::Duration(req.duration);
    goal.trajectory.points.push_back(point);
    goal.trajectory.header.stamp = ros::Time::now();

    client.sendGoal(goal);
    client.waitForResult();

    res.success = true;
    res.message = "Joint " + req.joint_name + " moved to " + std::to_string(req.target_position);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_single_joint_server");
    ros::NodeHandle nh;

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);
    ros::ServiceServer service = nh.advertiseService("move_single_joint", handle_request);

    ROS_INFO("Ready to receive joint move requests.");
    ros::spin();
    return 0;
}
