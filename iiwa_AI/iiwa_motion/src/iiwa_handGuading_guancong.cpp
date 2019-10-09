/* 
owner:  Clarence , 2019.July 
project name:   MotionROS
use iiwa_ros to control iiwa
*/
#include "ros/ros.h"
#include "std_msgs/String.h"


#include "iiwa_msgs/ControlMode.h"
#include "iiwa_msgs/JointImpedanceControlMode.h"
#include "iiwa_msgs/ConfigureControlMode.h"


#include "iiwa_ros/service/control_mode.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_handGuading_guancong_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");

    iiwa_msgs::ConfigureControlMode config;

    config.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE; //iiwa_msgs/CartesianImpedanceControlMode cartesian_impedance
    

    iiwa_msgs::JointImpedanceControlMode handmodel;

    handmodel.joint_stiffness.a1=0;
    handmodel.joint_stiffness.a2=0;
    handmodel.joint_stiffness.a3=0;
    handmodel.joint_stiffness.a4=0;
    handmodel.joint_stiffness.a5=0;
    handmodel.joint_stiffness.a6=0;
    handmodel.joint_stiffness.a7=0;

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
    


    config.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL;

    return 0;
}