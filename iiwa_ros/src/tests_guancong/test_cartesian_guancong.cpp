
/* GuanCong 2019.09.19
 * For services is also very similar:
 * create the object, initialize it, call their function to use the service.
 *
 */
#include <iostream>
#include <csignal>

#include <ros/ros.h>
#include <iiwa_ros/iiwa_ros.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose_linear.hpp>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static bool quit{false};

void signalHandler(int /*unused*/) { quit = true; }

int main(int argc, char **argv)
{
  // get roslauch param
  ros::init(argc, argv, "show_param");

  ros::NodeHandle nh;

  double direction_x_add;
  double direction_y_add;
  double direction_z_add;
  double rotation_x;
  double rotation_y;
  double rotation_z;
  double command_cartisian_mode;
  
  nh.getParam("CartesianPoseState_1569786053275419313/direction_x_add", direction_x_add);
  nh.getParam("CartesianPoseState_1569786053275419313/direction_y_add", direction_y_add);
  nh.getParam("CartesianPoseState_1569786053275419313/direction_z_add", direction_z_add);
  nh.getParam("rotation_x", rotation_x);
  nh.getParam("rotation_y", rotation_y);
  nh.getParam("rotation_z", rotation_z);
  nh.getParam("command_cartisian_mode", command_cartisian_mode);

  ROS_INFO("parameter is finish load................. ");


  iiwa_ros::state::CartesianPose pose_state;
  iiwa_ros::command::CartesianPose  pose_command;
  pose_command.init("iiwa");  
  pose_state.init("iiwa"); // That is the namespace under which the topic lives.

  ROS_INFO("init is finish................. "); 


  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Signal handlers.
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);

  ROS_INFO("singal is finish................. "); 
  // Wait a bit, so that you can be sure the subscribers are connected.
  ros::Duration(0.5).sleep();


  iiwa_msgs::CartesianPose car_pose = pose_state.getPose();
  std::cout << "Pose: " << car_pose.poseStamped.pose.orientation.w << std::endl;

  ROS_INFO_STREAM(
    std::to_string(car_pose.poseStamped.pose.position.x)
        << " " << std::to_string(car_pose.poseStamped.pose.position.y) << " " << std::to_string(car_pose.poseStamped.pose.position.z)
        << " " << std::to_string(car_pose.poseStamped.pose.orientation.x) << " " << std::to_string(car_pose.poseStamped.pose.orientation.y)
        << " " << std::to_string(car_pose.poseStamped.pose.orientation.z) << " " << std::to_string(car_pose.poseStamped.pose.orientation.w)
        << std::endl;);
  
  ROS_INFO("get current pose is finish................. "); 

  ros::Duration(0.1).sleep();


  //send a new Cartesian pose
  geometry_msgs::PoseStamped command_pose;
  command_pose = car_pose.poseStamped;
  command_pose.pose.position.x = car_pose.poseStamped.pose.position.x+direction_x_add;
  command_pose.pose.position.y = car_pose.poseStamped.pose.position.y+direction_y_add;
  command_pose.pose.position.z = car_pose.poseStamped.pose.position.z+direction_z_add;
  command_pose.pose.orientation.x = car_pose.poseStamped.pose.orientation.x;
  command_pose.pose.orientation.y = car_pose.poseStamped.pose.orientation.y;
  command_pose.pose.orientation.z = car_pose.poseStamped.pose.orientation.z;
  command_pose.pose.orientation.w = car_pose.poseStamped.pose.orientation.w;

  pose_command.setPose(command_pose);


  //  excute rotation command
  double rotation_x_radian = rotation_x * 180/M_PI;
  double rotation_y_radian = rotation_y * 180/M_PI;
  double rotation_z_radian = rotation_z * 180/M_PI;

  tf2::Quaternion q_orig,q_rot,q_new;

  tf2::convert(command_pose.pose.orientation,q_orig);
  q_rot.setRPY(rotation_x_radian,rotation_y_radian,rotation_z_radian);
  // world frame
  // q_new = q_rot*q_orig;
  // TCP frame
  q_new = q_orig*q_rot;      
  q_new.normalize();

  tf2::convert(q_new,command_pose.pose.orientation);

  pose_command.setPose(command_pose);

  ros::Duration(2).sleep();

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Bye!" << std::endl;

	ros::shutdown();

  return 0;
}
