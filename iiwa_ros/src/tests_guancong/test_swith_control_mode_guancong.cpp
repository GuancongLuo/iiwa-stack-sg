
/* GuanCong 2019.09.19
 * For services is also very similar:
 * create the object, initialize it, call their function to use the service.
 *
 */
#include <iostream>
#include <csignal>

#include <ros/ros.h>
#include <iiwa_ros/iiwa_ros.hpp>
#include <iiwa_ros/contorl_mode.hpp>

#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose_linear.hpp>
#include <geometry_msgs/PoseStamped.h>

static bool quit{false};

void signalHandler(int /*unused*/) { quit = true; }

int main()
{
  iiwa_ros::state::CartesianPose pose_state;
  iiwa_ros::command::CartesianPose  pose_command;

  pose_state.init("iiwa"); // That is the namespace under which the topic lives.
   pose_command.init("iiwa");

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Signal handlers.
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);

  // Wait a bit, so that you can be sure the subscribers are connected.
  ros::Duration(0.5).sleep();

  while (!quit)
  {
      //get current pose
      iiwa_msgs::CartesianPose car_pose = pose_state.getPose();
      std::cout << "Pose: " << car_pose.poseStamped.pose.orientation.w << std::endl;

      ROS_INFO_STREAM(
        std::to_string(car_pose.poseStamped.pose.position.x)
            << " " << std::to_string(car_pose.poseStamped.pose.position.y) << " " << std::to_string(car_pose.poseStamped.pose.position.z)
            << " " << std::to_string(car_pose.poseStamped.pose.orientation.x) << " " << std::to_string(car_pose.poseStamped.pose.orientation.y)
            << " " << std::to_string(car_pose.poseStamped.pose.orientation.z) << " " << std::to_string(car_pose.poseStamped.pose.orientation.w)
            << std::endl;);
      ros::Duration(0.1).sleep();

      //send a new Cartesian pose
      geometry_msgs::PoseStamped command_pose;
      command_pose = car_pose.poseStamped;
      command_pose.pose.position.x = 0.5;
      command_pose.pose.position.y = 0.3;
      command_pose.pose.position.z = 0.6;
      command_pose.pose.orientation.x = 0;
      command_pose.pose.orientation.y = 0.9238795;
      command_pose.pose.orientation.z = 0;
      command_pose.pose.orientation.w = 0.3826834;

       pose_command.setPose(command_pose);
  }

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Bye!" << std::endl;

  return 0;
}
