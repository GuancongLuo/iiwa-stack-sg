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
            cur_object_cam.header.frame_id = camera_frame;
            cur_object_cam.pose = getPose();
            tf::Stamped<tf::Pose> st;
            tf::poseStampedMsgToTF(cur_object_cam, st);
            tf_sender.sendTransform(tf::StampedTransform(st, ros::Time::now(), camera_frame, "object_frame"));
            std::cout << "Publish tf tree================\n";
            ros::spinOnce();
            ros::Duration(1).sleep();
        }
    }

    void setPose(geometry_msgs::Pose pose)
    {
        mutex_pose.lock();
        pose_ = pose;
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
    std::string camera_frame;
    tf::TransformBroadcaster tf_sender;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<motion_msgs::SetAbsolutePose>("/iiwa_motion/absolute_motion");

    // Open a new thread to object detection and publish tf
    ObjectDetectDummy obj;
    std::thread obj_publish_thread(&ObjectDetectDummy::run, &obj);

    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //current object pose wrt camera, 20cm in z
    geometry_msgs::PoseStamped cur_pose;
    cur_pose.header.frame_id = "iiwa_link_camera";
    cur_pose.pose.position.x = 0.0;
    cur_pose.pose.position.y = 0.0;
    cur_pose.pose.position.z = 0.3;
    cur_pose.pose.orientation.x = 0;
    cur_pose.pose.orientation.y = 0;
    cur_pose.pose.orientation.z = 0;
    cur_pose.pose.orientation.w = 1;
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
    command_pose.pose.orientation.y = 1;
    command_pose.pose.orientation.z = 0;
    command_pose.pose.orientation.w = 0;
    pose_command.setPose(command_pose);
    std::cout << "iiwa is in the current position!--------\n";
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
    srv.request.motion.pose_des = des_pose;
    std::cout <<"Start to call ros service /iiwa_motion/absolute_motion......\n";
    if (client.call(srv))
    {
        ROS_INFO("Successful to call /iiwa_motion/absolute_motion");
//        return srv.response.success;
        obj.setPose(des_pose.pose);
        std::cout << "iiwa is in the desreid position!--------\n";
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

    while(true)
    {
        ros::Duration(1).sleep();
    }

    spinner.stop();
    std::cerr << "Bye!" << std::endl;
    return 0;
}
