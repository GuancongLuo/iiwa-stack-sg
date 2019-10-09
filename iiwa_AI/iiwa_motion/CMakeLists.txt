cmake_minimum_required(VERSION 3.5)
project(iiwa_motion)

# ???????? How to know add this? Just copy
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compiler flags - compile with warning and debugging support
set(CMAKE_CXX_FLAGS "-Wall -g -fPIC -std=c++0x")

# cmake_modules ??? what is this facter? Just Copy
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  tf
  tf_conversions
  geometry_msgs
  eigen_conversions
  iiwa_ros
  iiwa_msgs
  motion_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	
        CATKIN_DEPENDS roscpp tf tf_conversions geometry_msgs eigen_conversions iiwa_msgs motion_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

# the node to open ros services
add_executable(iiwa_motion_node
              include/MotionROS.h
              src/MotionROS.cpp
              src/iiwa_move.cpp
)
target_link_libraries(iiwa_motion_node ${catkin_LIBRARIES} )

# test how to move the robot according to object pose
add_executable(test_command_robot
               src/test_command_robot.cpp
)
target_link_libraries(test_command_robot ${catkin_LIBRARIES} )

# test how to subsript the robot message
add_executable(iiwa_subsrciber
               src/iiwa_subsrciber.cpp
)
target_link_libraries(iiwa_subsrciber ${catkin_LIBRARIES} )

# test how to subsript the robot message
add_executable(iiwa_handGuading_guancong
               src/iiwa_handGuading_guancong.cpp
)
target_link_libraries(iiwa_handGuading_guancong ${catkin_LIBRARIES} )

add_executable(iiwa_command_test_guancong
               src/iiwa_command_test_guancong.cpp
)
target_link_libraries(iiwa_command_test_guancong ${catkin_LIBRARIES} )
## Add dependence to the motion_msg module for the library
# add_dependencies(iiwa_motion_node motion_msgs_generate_messages_cpp)
