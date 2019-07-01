/* Cai 2019.06.28
 * The test is showing an example how to call iiwa ros services
 * 1: send the object current and desired poses;
 * 2: call ros services which defined in MotionRos.cpp, to move robot;
 *
 */
#include <iostream>
#include <csignal>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iiwa_ros/iiwa_ros.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>

#include <motion_msgs/AbsoluteMotion.h>
#include <motion_msgs/SetAbsolutePose.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<motion_msgs::SetAbsolutePose>("/iiwa_motion/absolute_motion");

    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //current object pose wrt camera, 20cm in z
    geometry_msgs::PoseStamped cur_pose;
    cur_pose.header.frame_id = "camera_frame";
    cur_pose.pose.position.x = 0.0;
    cur_pose.pose.position.y = 0.0;
    cur_pose.pose.position.z = 0.2;
    cur_pose.pose.orientation.x = 0;
    cur_pose.pose.orientation.y = 0;
    cur_pose.pose.orientation.z = 0;
    cur_pose.pose.orientation.w = 1;

    //desired object pose wrt camera: the same point
    geometry_msgs::PoseStamped des_pose = cur_pose;
    des_pose.pose.position.z = 0.0;

    // ***************************************************************
    // move robot to current position first
    iiwa_ros::command::CartesianPose pose_command;
    iiwa_ros::state::CartesianPose pose_state;
    pose_command.init("iiwa");
    pose_state.init("iiwa");
    ros::Duration(1).sleep();

    // move robot to current pose
    geometry_msgs::PoseStamped command_pose;
    command_pose.header.frame_id = "iiwa_link_0";
    command_pose.pose.position.x = 0.5;
    command_pose.pose.position.y = 0.3;
    command_pose.pose.position.z = 0.6;
    command_pose.pose.orientation.x = 0;
    command_pose.pose.orientation.y = 1;
    command_pose.pose.orientation.z = 0;
    command_pose.pose.orientation.w = 0;
    pose_command.setPose(command_pose);
    ros::Duration(5).sleep(); //wait for the robot finish movement

    // print current robot pose
    iiwa_msgs::CartesianPose car_pose = pose_state.getPose();
    ROS_INFO_STREAM(
      std::to_string(car_pose.poseStamped.pose.position.x)
          << " " << std::to_string(car_pose.poseStamped.pose.position.y) << " " << std::to_string(car_pose.poseStamped.pose.position.z)
          << " " << std::to_string(car_pose.poseStamped.pose.orientation.x) << " " << std::to_string(car_pose.poseStamped.pose.orientation.y)
          << " " << std::to_string(car_pose.poseStamped.pose.orientation.z) << " " << std::to_string(car_pose.poseStamped.pose.orientation.w)
          << std::endl;);
    ros::Duration(0.1).sleep();
    // ***************************************************************

    // call the ros service
    motion_msgs::SetAbsolutePose srv;
    srv.request.motion.motion_name ="top";
    srv.request.motion.pose_cur = cur_pose;
    srv.request.motion.pose_cur = des_pose;
    std::cout << "pass ----- " << srv.request.motion.pose_cur.pose.position.z << std::endl;
    if (client.call(srv))
    {
        ROS_INFO("/iiwa_motion/absolute_motion");
//        return srv.response.success;
    }
    else
    {
        ROS_ERROR("Failed to call service /iiwa_motion/absolute_motion");
        return 1;
    }

    // print after robot pose
    car_pose = pose_state.getPose();
    ROS_INFO_STREAM(
    std::to_string(car_pose.poseStamped.pose.position.x)
          << " " << std::to_string(car_pose.poseStamped.pose.position.y) << " " << std::to_string(car_pose.poseStamped.pose.position.z)
          << " " << std::to_string(car_pose.poseStamped.pose.orientation.x) << " " << std::to_string(car_pose.poseStamped.pose.orientation.y)
          << " " << std::to_string(car_pose.poseStamped.pose.orientation.z) << " " << std::to_string(car_pose.poseStamped.pose.orientation.w)
          << std::endl;);
    ros::Duration(0.1).sleep();

    spinner.stop();
    std::cerr << "Bye!" << std::endl;
    return 0;
}
