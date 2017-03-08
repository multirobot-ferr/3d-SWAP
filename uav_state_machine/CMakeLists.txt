cmake_minimum_required(VERSION 2.8.3)
project(uav_state_machine)


set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ---- Configurable options ----
# Pass this options to CMake in command line to select specific functionality for this project
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	option(GRVCQUAD_USE_ROS "Enable ros-specific code and interfaces" ON)
else()
	option(GRVCQUAD_USE_ROS "Enable ros-specific code and interfaces" OFF)
endif()

# --- Compiler specific options ---
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

if(GRVCQUAD_USE_ROS)
	add_definitions(-DGRVC_USE_ROS)

	find_package(catkin REQUIRED COMPONENTS
			geometry_msgs
			mavros_msgs
            		sensor_msgs
			std_msgs
		    	roscpp
			grvc_quadrotor_uav
			grvc_quadrotor_utils
			message_generation)
			## Generate services in the 'srv' folder
	 add_service_files(
		   FILES
		   target_service.srv
		   takeoff_service.srv
		   waypoint_service.srv
		   land_service.srv
		   magnetize_service.srv
	 )
	add_message_files(
		   FILES
		   candidate.msg
		   candidate_list.msg
		   uav_state.msg
	 )

	## Generate added messages and services with any dependencies listed here
	 generate_messages(
		   DEPENDENCIES
		   std_msgs
		   geometry_msgs
	 )

	catkin_package(CATKIN_DEPENDS
			message_runtime
			geometry_msgs
			mavros_msgs
            		sensor_msgs
			std_msgs
		        roscpp
			grvc_quadrotor_uav
			grvc_quadrotor_utils)

	include_directories(
		include
		${catkin_INCLUDE_DIRS})
	link_libraries(${catkin_LIBRARIES})

	add_library(catching_device src/catching_device.cpp src/serial_catching_device.cpp)

endif(GRVCQUAD_USE_ROS)

## Test SerialCatchingDevice
add_executable(test_serial_catching_device src/test_serial_catching_device.cpp)
target_include_directories(test_serial_catching_device PUBLIC "include")
target_link_libraries(test_serial_catching_device catching_device)

## Declare a C++ executable
file(GLOB_RECURSE UAV_STATE_MACHINE_SOURCE_FILES "src/main.cpp" "src/state_machine.cpp" "include/*.h" )
add_executable(uav_state_machine ${UAV_STATE_MACHINE_SOURCE_FILES})
target_include_directories(uav_state_machine PUBLIC "include")
target_link_libraries(uav_state_machine LINK_PUBLIC catching_device uav_server)
link_libraries(uav_state_machine LINK ${GRVCQUADROTOR_LIBRARIES})
