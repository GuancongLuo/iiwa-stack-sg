cmake_minimum_required(VERSION 3.5)
project(iiwa_gui)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compiler flags - compile with warning and debugging support
set(CMAKE_CXX_FLAGS "-Wall -g -fPIC -std=c++0x")

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

find_package(Qt5 COMPONENTS Core Gui Network OpenGL Widgets)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
        INCLUDE_DIRS src
	LIBRARIES	
        CATKIN_DEPENDS roscpp tf tf_conversions geometry_msgs eigen_conversions iiwa_msgs motion_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${QT_INCLUDES}
)

set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES} ${Qt5Network_LIBRARIES} ${Qt5Widgets_LIBRARIES})

SET(
    VIZ_UI
    src/iiwaGUI.ui
)

set(VIZ_HDRS
    src/iiwaGUI.h
)

set(VIZ_SRCS
    src/iiwaGUI.cpp
)


add_executable(iiwa_gui_node
                ${VIZ_UI}
                ${VIZ_HDRS}
                ${VIZ_SRCS}
                src/iiwa_gui_node.cpp
                )
target_link_libraries(iiwa_gui_node ${catkin_LIBRARIES} ${QT_LIBRARIES})

