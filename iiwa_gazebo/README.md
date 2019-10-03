
# roslaunch iiwa_gazebo_with_sunrise.launch 

## $ rosnode list
-   /gazebo
-  /gazebo_gui
- /iiwa/controller_spawner
- /iiwa/iiwa_sunrise
- /iiwa/robot_state_publisher

---
#### $ rosnode info /iiwa/iiwa_sunrise 
---

Node [/iiwa/iiwa_sunrise]
Publications: 
 * /iiwa/PositionJointInterface_trajectory_controller/command [trajectory_msgs/JointTrajectory]
 * /iiwa/state/CartesianPose [iiwa_msgs/CartesianPose]
 * /iiwa/state/JointPosition [iiwa_msgs/JointPosition]

Subscriptions: 
 * /iiwa/command/CartesianPose [unknown type]
 * /iiwa/command/CartesianPoseLin [unknown type]
 * /iiwa/command/JointPosition [unknown type]
 * /iiwa/command/redundancy [unknown type]
 * /iiwa/joint_states [sensor_msgs/JointState]

Services: 
 * /iiwa/configuration/ConfigureControlMode
 * /iiwa/configuration/setSmartServoLimits
 * /iiwa/configuration/setSmartServoLinLimits
 * /iiwa/iiwa_sunrise/get_loggers
 * /iiwa/iiwa_sunrise/set_logger_level



---
#### $ rosnode info /iiwa/robot_state_publisher 
---

Node [/iiwa/robot_state_publisher]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /iiwa/joint_states [sensor_msgs/JointState]

Services: 
 * /iiwa/robot_state_publisher/get_loggers
 * /iiwa/robot_state_publisher/set_logger_level


---
## $ rostopic list
---
1. gazebo

        /gazebo/link_states
        /gazebo/model_states
        /gazebo/parameter_descriptions
        /gazebo/parameter_updates
        /gazebo/set_link_state
        /gazebo/set_model_state
        /gazebo_gui/parameter_descriptions
        /gazebo_gui/parameter_updates

2. ros action controll

        /iiwa/PositionJointInterface_trajectory_controller/command
        /iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory/cancel
        /iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory/feedback
        /iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory/goal
        /iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory/result
        /iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory/status
        /iiwa/PositionJointInterface_trajectory_controller/state

3. iiwa command

        /iiwa/command/CartesianPose
        /iiwa/command/CartesianPoseLin
        /iiwa/command/JointPosition
        /iiwa/command/redundancy

4. iiwa state

        /iiwa/joint_states
        /iiwa/state/CartesianPose
        /iiwa/state/CartesianWrench
        /iiwa/state/JointPosition

5. ros tf

        /statistics
        /tf
        /tf_static


---
#### $ rostopic info /iiwa/joint_states 
---
    Type: sensor_msgs/JointState

