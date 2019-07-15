## IIWA STACK

* **iiwa_stack:**  the full origial one: https://github.com/IFL-CAMP/iiwa_stack, check readme_origin.md

            I add my own package iiwa_motion on top of it, and also modified some files.


#
###
This packages contained in this repository are :

*  **iiwa_AI**: the ROS package which use the iwa_ros services (added by Cai)

*  **iiwa_control**: contains the joint and trajectory controllers used by MoveIt! and Gazebo.

*  **iiwa_description**: URDF for both KUKA LBR IIWA R800 and R820.

*  **iiwa_gazebo**: launch files to run a Gazebo simulation.

*  **iiwa_hw**: implements the ROS hardware interface and the communication interface with the real robot (using iiwa_ros).

*  **iiwa_moveit**: a MoveIt! configuration for controlling the robot (either a Gazebo simulation or a real one).

*  **iiwa_msgs**: creates ROS messages to be used for communication with a real robot. 

*  **iiwa_ros**: an interface to send and receive messages defined in iiwa_msgs to and from a real robot.

*  **iiwa_ros_java**: the ROSJava interface to use on SunriseApplications - it allows to send and receive messages defined in iiwa_msgs.


#
###Acknowledgements: 

**You yangwei:**

* **iiwa_ros_interface:**  only have the simple iiwa ros interterface (joint control);

* **iiwa_industrial_stack:**  simple iiwa ros interterface +  gazebo simulation



