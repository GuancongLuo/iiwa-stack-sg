/* Guancong 2019.09.19
 * The test is showing an example how to use iiwa/command topic. 
 * 1: send the object current and desired poses;
 * 2: call ros services which defined in MotionRos.cpp, to move robot;
 * 
 * 3.for the Relative Motion test
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

#include <cmath>
// publicer
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "relative_motion_Z");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

    // ROS spinner.
    ros::AsyncSpinner spinner(2);
    spinner.start();

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */


    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    // ***************************************************************
    // move robot to current position first
    iiwa_ros::command::CartesianPose pose_command;
    iiwa_ros::state::CartesianPose pose_state;
    pose_command.init("iiwa");
    pose_state.init("iiwa");
    ros::Duration(1).sleep();

    // move robot to current pose
    geometry_msgs::PoseStamped command_pose;
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


    while(true)
    {
        ros::Duration(1).sleep();
    }

    spinner.stop();
    std::cerr << "Bye!" << std::endl;
    return 0;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */


    ros::spinOnce();




  return 0;
}



