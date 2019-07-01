

# Run gazebo simulation with sunrice
    $ roslaunch iiwa_gazebo iiwa_gazebo_with_sunrise.launch
    $ rosrun iiwa_ros iiwa_ros_joint (joint control)
    $ rosrun iiwa_ros iiwa_ros_cartesian (cartesian control)

#### Gazobo topics:
* Subscribers:

		/iiwa/command/CartesianPose
		/iiwa/command/CartesianPoseLin
		/iiwa/command/JointPosition
		/iiwa/command/redundancy
		/iiwa/joint_states
	
* Publishers:

		/iiwa/state/CartesianPose
		/iiwa/state/CartesianWrench
		/iiwa/state/JointPosition

I  add one more ros topic to publish the robot joint posiiton: map from **/iiwa/joint_states to /iiwa/state/JointPosition**


   
#### Joint control in simulation:

	1.topic: /iiwa/state/JointPosition
	Publishers: 
	* /iiwa/iiwa_sunrise 
	Subscribers: 
	* /JointPositionState    ------> get in ros class:  iiwa_ros::state::JointPosition
	
	
	2. topic:  /iiwa/command/JointPosition
	Publishers: 
	* /JointPositionState   <------- set by ros class:   iiwa_ros::command::JointPosition 
	Subscribers: 
	*/iiwa/iiwa_sunrise


#### Cartesian control in simulation:

     Use topics to get and set:
     
     /iiwa/state/CartesianPose                ---->   [iiwa_msgs/CartesianPose] 
     /iiwa/command/CartesianPose       <-----  [geometry_msgs/PoseStamped]
     
 **Problem: **
       
    error: Could not process inbound connection: topic types do not match:** [iiwa_msgs/CartesianPose] vs. [geometry_msgs/PoseStamped]**{'topic': '/iiwa/state/CartesianPose'.  I modified the simulation file: iiwa_sunrise.py to solve this problem.
 
