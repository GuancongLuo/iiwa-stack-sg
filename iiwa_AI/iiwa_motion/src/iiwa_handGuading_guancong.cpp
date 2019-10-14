/* 
owner:  Clarence , 2019.July 
project name:   MotionROS
use iiwa_ros to control iiwa
*/

#include <iostream>
#include <csignal>

#include "ros/ros.h"
#include "std_msgs/String.h"


#include "iiwa_msgs/ControlMode.h"
#include "iiwa_msgs/JointImpedanceControlMode.h"
#include "iiwa_msgs/ConfigureControlMode.h"
#include <iiwa_ros/state/cartesian_pose.hpp>


#include "iiwa_ros/service/control_mode.hpp"

static bool quit{false};

void signalHandler(int /*unused*/) { quit = true; }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_handGuading_guancong_client");

    iiwa_ros::state::CartesianPose pose_state;
    pose_state.init("iiwa");
    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Signal handlers.
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);
    signal(SIGHUP, signalHandler);

    // Wait a bit, so that you can be sure the subscribers are connected.
    ros::Duration(0.5).sleep();

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");

    iiwa_msgs::ConfigureControlMode config;

    config.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE; //iiwa_msgs/CartesianImpedanceControlMode cartesian_impedance
    

    iiwa_msgs::JointImpedanceControlMode handmodel;

    handmodel.joint_stiffness.a1=10;
    handmodel.joint_stiffness.a2=10;
    handmodel.joint_stiffness.a3=10;
    handmodel.joint_stiffness.a4=10;
    handmodel.joint_stiffness.a5=10;
    handmodel.joint_stiffness.a6=10;
    handmodel.joint_stiffness.a7=10;

    handmodel.joint_damping.a1=0.7;
    handmodel.joint_damping.a2=0.7;
    handmodel.joint_damping.a3=0.7;
    handmodel.joint_damping.a4=0.7;
    handmodel.joint_damping.a5=0.7;
    handmodel.joint_damping.a6=0.7;
    handmodel.joint_damping.a7=0.7;


    if (client.call(config))
    {
        if (!config.response.success)
        {
            ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
        }
        else
        {
            ROS_INFO_STREAM("SmartServo Service successfully called.");
        }
    }
    else
    {
        ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
    }
    
    ros::Duration(2).sleep();
    while (!quit)
    {
      iiwa_msgs::CartesianPose car_pose = pose_state.getPose();
      ROS_INFO_STREAM(
        std::to_string(car_pose.poseStamped.pose.position.x)
            << " " << std::to_string(car_pose.poseStamped.pose.position.y) << " " << std::to_string(car_pose.poseStamped.pose.position.z)
            << " " << std::to_string(car_pose.poseStamped.pose.orientation.x) << " " << std::to_string(car_pose.poseStamped.pose.orientation.y)
            << " " << std::to_string(car_pose.poseStamped.pose.orientation.z) << " " << std::to_string(car_pose.poseStamped.pose.orientation.w)
            << std::endl;);
      ros::Duration(0.1).sleep();
    }

    config.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL; //iiwa_msgs/CartesianImpedanceControlMode cartesian_impedance

    if (client.call(config))
    {
        if (!config.response.success)
        {
            ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
        }
        else
        {
            ROS_INFO_STREAM("SmartServo Service successfully called.");
        }
    }
    else
    {
        ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
    }
    // config.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL;

    return 0;
}