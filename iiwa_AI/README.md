This package is design for collaborative AI project.

# Run gazebo simulation with sunrice

* run gazebo:
       
        $ roslaunch iiwa_gazebo iiwa_gazebo_with_sunrise.launch
        
* run iiwa ros services: 

        $ rosrun iiwa_motion iiwa_motion_node
        $ rosrun iiwa_motion tf_server.py
        or
        $ roslaunch iiwa_motion iiwa_sim.launch

 * test command iiwa, which should be called by other package (e.g. high-level object pose) : 
 
        $ rosrun iiwa_motion test_command_robot