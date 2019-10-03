This package is design for collaborative AI project.

# Run gazebo simulation with sunrice

* run gazebo:
       
        $ roslaunch iiwa_gazebo iiwa_gazebo_with_sunrise.launch
        
* run iiwa ros services: 

        $ roslaunch iiwa_motion iiwa_sim.launch

 * test command iiwa, which should be called by other package (e.g. high-level object pose) : 
 
        $ rosrun iiwa_motion test_command_robot
        
 * Visualization:
     
        $rviz
        
        
        
### Send command use GUI:



The real iiwa parament
# roscore and the real robot

## $ rosnode list
/iiwa/iiwa_action_server
/iiwa/iiwa_configuration
/iiwa/iiwa_publisher
/iiwa/iiwa_subscriber
---
#### $ rosnode info /iiwa/iiwa_subscriber
Node [/iiwa/iiwa_subscriber]
Publications: 
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /iiwa/command/CartesianPose [unknown type]
 * /iiwa/command/CartesianPoseLin [unknown type]
 * /iiwa/command/CartesianVelocity [unknown type]
 * /iiwa/command/JointPosition [unknown type]
 * /iiwa/command/JointPositionVelocity [unknown type]
 * /iiwa/command/JointVelocity [unknown type]
 * /tf [unknown type]
 * /tf_static [unknown type]

Services: 
 * /iiwa/configuration/ConfigureControlMode
 * /iiwa/configuration/setEndpointFrame
 * /iiwa/configuration/setPTPCartesianLimits
 * /iiwa/configuration/setPTPJointLimits
 * /iiwa/configuration/setSmartServoLimits
 * /iiwa/configuration/setSmartServoLinLimits
 * /iiwa/configuration/setSpeedOverride
 * /iiwa/configuration/setWorkpiece
 * /iiwa/state/timeToDestination

---

---
#### $ rosnode info /iiwa/iiwa_publisher
Node [/iiwa/iiwa_publisher]
Publications: 
 * /iiwa/joint_states [sensor_msgs/JointState]
 * /iiwa/state/CartesianPose [iiwa_msgs/CartesianPose]
 * /iiwa/state/CartesianWrench [iiwa_msgs/CartesianWrench]
 * /iiwa/state/DestinationReached [std_msgs/Time]
 * /iiwa/state/ExternalJointTorque [iiwa_msgs/JointTorque]
 * /iiwa/state/JointPosition [iiwa_msgs/JointPosition]
 * /iiwa/state/JointPositionVelocity [iiwa_msgs/JointPositionVelocity]
 * /iiwa/state/JointTorque [iiwa_msgs/JointTorque]
 * /iiwa/state/JointVelocity [iiwa_msgs/JointVelocity]
 * /iiwa/state/buttonEvent [std_msgs/String]
 * /rosout [rosgraph_msgs/Log]
---

---
#### $ rosnode info /iiwa/iiwa_action_server
Node [/iiwa/iiwa_action_server]
Publications: 
 * /iiwa/action/move_along_spline/feedback [iiwa_msgs/MoveAlongSplineActionFeedback]
 * /iiwa/action/move_along_spline/result [iiwa_msgs/MoveAlongSplineActionResult]
 * /iiwa/action/move_along_spline/status [actionlib_msgs/GoalStatusArray]
 * /iiwa/action/move_to_cartesian_pose/feedback [iiwa_msgs/MoveToCartesianPoseActionFeedback]
 * /iiwa/action/move_to_cartesian_pose/result [iiwa_msgs/MoveToCartesianPoseActionResult]
 * /iiwa/action/move_to_cartesian_pose/status [actionlib_msgs/GoalStatusArray]
 * /iiwa/action/move_to_cartesian_pose_lin/feedback [iiwa_msgs/MoveToCartesianPoseActionFeedback]
 * /iiwa/action/move_to_cartesian_pose_lin/result [iiwa_msgs/MoveToCartesianPoseActionResult]
 * /iiwa/action/move_to_cartesian_pose_lin/status [actionlib_msgs/GoalStatusArray]
 * /iiwa/action/move_to_joint_position/feedback [iiwa_msgs/MoveToJointPositionActionFeedback]
 * /iiwa/action/move_to_joint_position/result [iiwa_msgs/MoveToJointPositionActionResult]
 * /iiwa/action/move_to_joint_position/status [actionlib_msgs/GoalStatusArray]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /iiwa/action/move_along_spline/cancel [unknown type]
 * /iiwa/action/move_along_spline/goal [unknown type]
 * /iiwa/action/move_to_cartesian_pose/cancel [unknown type]
 * /iiwa/action/move_to_cartesian_pose/goal [unknown type]
 * /iiwa/action/move_to_cartesian_pose_lin/cancel [unknown type]
 * /iiwa/action/move_to_cartesian_pose_lin/goal [unknown type]
 * /iiwa/action/move_to_joint_position/cancel [unknown type]
 * /iiwa/action/move_to_joint_position/goal [unknown type]

Services: None

---





## $ rostopic list

2. ros action controll

/iiwa/action/move_along_spline/cancel
/iiwa/action/move_along_spline/feedback
/iiwa/action/move_along_spline/goal
/iiwa/action/move_along_spline/result
/iiwa/action/move_along_spline/status

/iiwa/action/move_to_cartesian_pose/cancel
/iiwa/action/move_to_cartesian_pose/feedback
/iiwa/action/move_to_cartesian_pose/goal
/iiwa/action/move_to_cartesian_pose/result
/iiwa/action/move_to_cartesian_pose/status

/iiwa/action/move_to_cartesian_pose_lin/cancel
/iiwa/action/move_to_cartesian_pose_lin/feedback
/iiwa/action/move_to_cartesian_pose_lin/goal
/iiwa/action/move_to_cartesian_pose_lin/result
/iiwa/action/move_to_cartesian_pose_lin/status

/iiwa/action/move_to_joint_position/cancel
/iiwa/action/move_to_joint_position/feedback
/iiwa/action/move_to_joint_position/goal
/iiwa/action/move_to_joint_position/result
/iiwa/action/move_to_joint_position/status


3. iiwa command

/iiwa/command/CartesianPose
/iiwa/command/CartesianPoseLin
/iiwa/command/CartesianVelocity

/iiwa/command/JointPosition
/iiwa/command/JointPositionVelocity
/iiwa/command/JointVelocity


4. iiwa state

/iiwa/joint_states
/iiwa/state/CartesianPose
/iiwa/state/CartesianWrench
/iiwa/state/DestinationReached
/iiwa/state/ExternalJointTorque
/iiwa/state/JointPosition
/iiwa/state/JointPositionVelocity
/iiwa/state/JointTorque
/iiwa/state/JointVelocity
/iiwa/state/buttonEvent


5. ros tf

/tf
/tf_static

---
#### $ rostopic info /iiwa/joint_states 
---


## $ rosservice list

/iiwa/configuration/ConfigureControlMode
/iiwa/configuration/setEndpointFrame
/iiwa/configuration/setPTPCartesianLimits
/iiwa/configuration/setPTPJointLimits
/iiwa/configuration/setSmartServoLimits
/iiwa/configuration/setSmartServoLinLimits
/iiwa/configuration/setSpeedOverride
/iiwa/configuration/setWorkpiece
/iiwa/state/timeToDestination


