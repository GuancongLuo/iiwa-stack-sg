#ifndef MOTION_ROS_H
#define MOTION_ROS_H

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <iiwa_ros/iiwa_ros.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>

#include "motion_msgs/AbsoluteMotion.h"
#include "motion_msgs/RelativeMotion.h"
#include "motion_msgs/SetAbsolutePose.h"
#include "motion_msgs/SetRelativePose.h"
#include "motion_msgs/GetTransform.h"
#include "motion_msgs/TransformPose.h"

class MotionROS
{
    public:

        MotionROS(const ros::NodeHandle& nh_);
        ~MotionROS();

        /// \brief  Callback function for service
        bool set_absolute_callback(motion_msgs::SetAbsolutePose::Request& request,
                                   motion_msgs::SetAbsolutePose::Response& response);

        bool set_relative_callback(motion_msgs::SetRelativePose::Request& request,
                                   motion_msgs::SetRelativePose::Response& response);


    private:
        bool setAbsolutePose(motion_msgs::AbsoluteMotion& motion);
        bool setRelativePose(motion_msgs::RelativeMotion& motion);
        void printPoseStamp(geometry_msgs::PoseStamped& poseStamped);

        // tf_tools
        static geometry_msgs::PoseStamped getPoseWrtFrame(ros::NodeHandle &node, const std::string child_frame, const std::string frame);
        static geometry_msgs::PoseStamped transformPoseWrtFrame(ros::NodeHandle& node, const geometry_msgs::PoseStamped& pose, const std::string frame);

        //iiwa_ros states
        iiwa_ros::state::CartesianPose pose_state;
        iiwa_ros::command::CartesianPose pose_command;

        //robot end-effector pose wrt to base frame
        geometry_msgs::PoseStamped current_ef_pose;
        geometry_msgs::PoseStamped desired_ef_pose;

        ros::NodeHandle nh;
        ros::ServiceServer set_absolute_srv;
        ros::ServiceServer set_relative_srv;

        //calibrate where is the camera on robot
        Eigen::Affine3d trans_camera_ef;

        //tf frame
        std::string camera_frame;
        std::string object_frame;
        std::string camera_ref_frame;
        std::string robot_base;

        tf::TransformBroadcaster tf_sender;
};

#endif  // MOTION_ROS_H
