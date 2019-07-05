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
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <motion_msgs/AbsoluteMotion.h>
#include <motion_msgs/SetAbsolutePose.h>
#include <motion_msgs/RelativeMotion.h>
#include <motion_msgs/SetRelativePose.h>

#include <mutex>
#include <thread>

class ObjectDetectDummy
{
  public:
    ObjectDetectDummy()
    {
        camera_frame = "iiwa_link_camera";
    }
    void run()
    {
        while(true)
        {
            //publish current object pose to tf wrt camera frame
            geometry_msgs::PoseStamped cur_object_cam;
            cur_object_cam.header.frame_id = "iiwa_link_0"; //object orientation is the same as the base frame
            cur_object_cam.pose = getPose();
            tf::Stamped<tf::Pose> st;
            tf::poseStampedMsgToTF(cur_object_cam, st);
            tf_sender.sendTransform(tf::StampedTransform(st, ros::Time::now(), "iiwa_link_0", "object_frame"));
            std::cout << "Publish tf tree================\n";
            ros::spinOnce();
            ros::Duration(2).sleep();
        }
    }

    void setPose(geometry_msgs::Pose pose)
    {
        mutex_pose.lock();
        pose_ = pose;
        //fake the pose: get current object pose wrt iiwa_link_0
        // Current object pose wrt base: iiwa_link_0 -- 0.711442 0.301436 0.387487 0.000083 0.000352 0.001049 0.999999
        // for real system, every time will get the object_cam, and cam_base
        pose_.position.x = 0.711442;
        pose_.position.y = 0.301436;
        pose_.position.z = 0.387487;
        pose_.orientation.x = 0;
        pose_.orientation.y = 0;
        pose_.orientation.z = 0;
        pose_.orientation.w = 1;
        mutex_pose.unlock();
    }

    geometry_msgs::Pose getPose()
    {
        geometry_msgs::Pose pose;
        mutex_pose.lock();
        pose = pose_;
        mutex_pose.unlock();
        return pose;
    }

    std::mutex mutex_pose;
    geometry_msgs::Pose pose_;
    tf::TransformBroadcaster tf_sender;
    std::string camera_frame;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_client");

    ros::NodeHandle n;
    ros::ServiceClient client_absolute = n.serviceClient<motion_msgs::SetAbsolutePose>("/iiwa_motion/absolute_motion");
    ros::ServiceClient client_relative = n.serviceClient<motion_msgs::SetRelativePose>("/iiwa_motion/relative_motion");

    // Open a new thread to object detection and publish tf
    ObjectDetectDummy obj;
    std::thread obj_publish_thread(&ObjectDetectDummy::run, &obj);

    // ROS spinner.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    //current object pose wrt camera, 30cm in z
    geometry_msgs::PoseStamped cur_pose;
    cur_pose.header.frame_id = "iiwa_link_camera";
    cur_pose.pose.position.x = 0.0;
    cur_pose.pose.position.y = 0.0;
    cur_pose.pose.position.z = 0.3;
    cur_pose.pose.orientation.x = 0;
    cur_pose.pose.orientation.y = -0.9238795;
    cur_pose.pose.orientation.z = 0;
    cur_pose.pose.orientation.w = 0.3826834;
    obj.setPose(cur_pose.pose);

    //desired object pose wrt camera: the same point
    geometry_msgs::PoseStamped des_pose = cur_pose;
    des_pose.pose.position.z = 0.1;

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
    command_pose.pose.orientation.y = 0.9238795;
    command_pose.pose.orientation.z = 0;
    command_pose.pose.orientation.w = 0.3826834;
    pose_command.setPose(command_pose);
    std::cout << "iiwa is in the current position!--------\n";
    ros::Duration(5).sleep(); //wait for the robot finish movement

    // print current robot pose
    iiwa_msgs::CartesianPose car_pose = pose_state.getPose();
    ROS_INFO_STREAM(
          car_pose.poseStamped.header.frame_id << " -- " << std::to_string(car_pose.poseStamped.pose.position.x)
          << " " << std::to_string(car_pose.poseStamped.pose.position.y) << " " << std::to_string(car_pose.poseStamped.pose.position.z)
          << " " << std::to_string(car_pose.poseStamped.pose.orientation.x) << " " << std::to_string(car_pose.poseStamped.pose.orientation.y)
          << " " << std::to_string(car_pose.poseStamped.pose.orientation.z) << " " << std::to_string(car_pose.poseStamped.pose.orientation.w)
          << std::endl;);
    ros::Duration(0.1).sleep();
    // ***************************************************************

    // call the ros service 1
//    std::cout <<"---------------------------------------\n";
//    motion_msgs::SetAbsolutePose srv;
//    srv.request.motion.motion_name ="top";
//    srv.request.motion.pose_cur = cur_pose;
//    srv.request.motion.pose_des = des_pose;
//    std::cout <<"Start to call ros service /iiwa_motion/absolute_motion......\n";
//    if (client_absolute.call(srv))
//    {
//        ROS_INFO("Successful to call /iiwa_motion/absolute_motion");
//        obj.setPose(des_pose.pose);
//    }
//    else
//    {
//        ROS_ERROR("Failed to call service /iiwa_motion/absolute_motion");
//        return 1;
//    }

    // call the ros service 2
    std::cout <<"---------------------------------------\n";
    motion_msgs::SetRelativePose srv2;
    srv2.request.motion.motion_name = "angle";
    srv2.request.motion.pose_cur = cur_pose;
    srv2.request.motion.percentage_des = -45.0;
    std::cout <<"Start to call ros service /iiwa_motion/relative_motion......\n";
    if (client_relative.call(srv2))
    {
        ROS_INFO("Successful to call /iiwa_motion/relative_motion");
        //obj.setPose(des_pose.pose);
    }
    else
    {
        ROS_ERROR("Failed to call service /iiwa_motion/relative_motion");
        return 1;
    }

    // print after robot pose
    std::cout <<"---------------------------------------\n";
    std::cout << "iiwa is in the desreid position!--------\n";
    car_pose = pose_state.getPose();
    ROS_INFO_STREAM(
    std::to_string(car_pose.poseStamped.pose.position.x)
          << " " << std::to_string(car_pose.poseStamped.pose.position.y) << " " << std::to_string(car_pose.poseStamped.pose.position.z)
          << " " << std::to_string(car_pose.poseStamped.pose.orientation.x) << " " << std::to_string(car_pose.poseStamped.pose.orientation.y)
          << " " << std::to_string(car_pose.poseStamped.pose.orientation.z) << " " << std::to_string(car_pose.poseStamped.pose.orientation.w)
          << std::endl;);

    while(true)
    {
        ros::Duration(1).sleep();
    }

    spinner.stop();
    std::cerr << "Bye!" << std::endl;
    return 0;
}
