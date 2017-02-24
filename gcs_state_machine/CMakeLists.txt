################################################################################
# GRVC MBZIRC vision
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
cmake_minimum_required (VERSION 2.6)
project(gcs_state_machine)

# ---- Configurable options ----
# Pass this options to CMake in command line to select specific functionality for this project
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    add_definitions(-DGRVC_USE_ROS)
endif()

# --- Compiler specific options ---
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -Wall)
endif()

# --- Catkin specific configuration for ros ---
find_package(catkin REQUIRED COMPONENTS
			geometry_msgs
			mavros_msgs
			sensor_msgs
			std_msgs
			roscpp
			grvc_quadrotor_uav
			grvc_quadrotor_utils
			uav_state_machine 
			message_generation)

add_message_files(	FILES
			gcs_state.msg )

generate_messages(	DEPENDENCIES
			std_msgs
			geometry_msgs )

catkin_package(		CATKIN_DEPENDS message_runtime
			geometry_msgs
			mavros_msgs
			sensor_msgs
			std_msgs
			roscpp
			grvc_quadrotor_uav
			grvc_quadrotor_utils)


include_directories(${catkin_INCLUDE_DIRS})

# --- Main executable, simple node running the uav service ---
set(APP_NAME gcs_state_machine)
file(GLOB_RECURSE GCSSTATEMACHINE_SOURCE_FILES "src/*.cpp" "include/*.h" "include/*.inl")

add_executable(${APP_NAME} ${GCSSTATEMACHINE_SOURCE_FILES})
target_include_directories(${APP_NAME} PUBLIC "include")

target_include_directories(${APP_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${APP_NAME} LINK_PUBLIC ${catkin_LIBRARIES})


