################################################################################
# GRVC Quadrotor
################################################################################
# The MIT License (MIT)
# 
# Copyright (c) 2016 GRVC University of Seville
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(visualization)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages	
find_package(
	catkin REQUIRED COMPONENTS  
			roscpp 
			geometry_msgs 
			visualization_msgs
        	image_transport 
			std_msgs 
			cv_bridge 
			uav_state_machine 
			tf 
			gazebo_ros
)

find_package(OpenCV REQUIRED COMPONENTS)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
	LIBRARIES	
	CATKIN_DEPENDS  
		roscpp 
		geometry_msgs 
		visualization_msgs
        image_transport 
		std_msgs 
		cv_bridge 
		uav_state_machine 
		tf 
		gazebo_ros
	DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

add_executable(visualizer_node src/visualizer_node.cpp)
add_dependencies(visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualizer_node ${catkin_LIBRARIES})



add_executable(createfileworld src/createfileworld_urdf.cpp)
target_link_libraries(createfileworld LINK_PUBLIC grvc_com)

add_executable(cameraview src/cameraview.cpp)
target_link_libraries(cameraview ${catkin_LIBRARIES})

add_executable(moveobject src/moveobject.cpp)
target_link_libraries(moveobject LINK_PUBLIC hal_server)
set_target_properties(moveobject PROPERTIES FOLDER "samples")

add_executable(moveall src/moveall.cpp)
target_link_libraries(moveall ${catkin_LIBRARIES})

add_executable(marker_stage src/marker_stage.cpp)
target_link_libraries(marker_stage ${catkin_LIBRARIES})

add_executable(marker_waypoints src/marker_waypoints.cpp)
target_link_libraries(marker_waypoints ${catkin_LIBRARIES})

add_executable(take_object src/take_object.cpp)
target_link_libraries(take_object ${catkin_LIBRARIES})

add_executable(moving_object src/moving_object.cpp)
target_link_libraries(moving_object ${catkin_LIBRARIES})

add_executable(gazebo_move src/gazebo_move.cpp)
target_link_libraries(gazebo_move ${catkin_LIBRARIES})
