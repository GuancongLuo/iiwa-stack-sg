/* Cai 2019.06.25
ROSSmartServo subscribes to the following topics for the following command modes:

Position control in joint space on /iiwa/command/JointPosition
Position control in cartesian space on /iiwa/command/CartesianPose
Velocity control in joint space on /iiwa/command/JointVelocity
Joint/Velocity commands on /iiwa/command/JointPositionVelocity the robot will reach the given destination at the given velocity
 *
 */
#include <iostream>
#include <csignal>

#include <ros/ros.h>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/state/joint_torque.hpp>
#include <iiwa_ros/command/joint_position.hpp>

static bool quit{false};

void signalHandler(int /*unused*/) { quit = true; }

int main() {
  iiwa_ros::state::JointPosition jp_state;
  iiwa_ros::state::JointTorque jt_state;
  iiwa_ros::command::JointPosition jp_command;

  jp_state.init("iiwa");
  jt_state.init("iiwa");
  jp_command.init("iiwa");

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Signal handlers.
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);

  // Wait a bit, so that you can be sure the subscribers are connected.
  ros::Duration(0.5).sleep();

  while (!quit) {
    auto joint_position_ = jp_state.getPosition();

    auto joint_torque = jt_state.getTorque();
    ROS_INFO_STREAM(
        std::to_string(joint_position_.position.a1)
            << " " << std::to_string(joint_position_.position.a2) << " " << std::to_string(joint_position_.position.a3)
            << " " << std::to_string(joint_position_.position.a4) << " " << std::to_string(joint_position_.position.a5)
            << " " << std::to_string(joint_position_.position.a6) << " " << std::to_string(joint_position_.position.a7)
            << std::endl;);
    ros::Duration(0.1).sleep();

    //sent a new joint pose
    iiwa_msgs::JointPosition joints;
    joints = joint_position_;
    joints.position.a1 = 1.57;
    joints.position.a4 = 1.57;
    jp_command.setPosition(joints);
  }

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Bye!" << std::endl;

  return 0;
}
