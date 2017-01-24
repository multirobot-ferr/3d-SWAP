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
			mavros
			mavros_msgs
		    roscpp
			grvc_quadrotor_uav
			message_generation)
			## Generate services in the 'srv' folder
	 add_service_files(
		   FILES
		   target_service.srv
		   takeoff_service.srv
		   land_service.srv
	 )

	## Generate added messages and services with any dependencies listed here
	 generate_messages(
		   DEPENDENCIES
		   std_msgs
	 )

	catkin_package(CATKIN_DEPENDS
			grvc_quadrotor_uav)

	include_directories(${catkin_INCLUDE_DIRS})
	link_libraries(${catkin_LIBRARIES})
endif(GRVCQUAD_USE_ROS)

## Declare a C++ executable
file(GLOB_RECURSE VISUALSERVOING_SOURCE_FILES "src/*.cpp" "include/*.h" "include/*.inl")
add_executable(uav_state_machine ${VISUALSERVOING_SOURCE_FILES})
target_include_directories(uav_state_machine PUBLIC "include")
target_link_libraries(uav_state_machine LINK_PUBLIC uav_server)
link_libraries(uav_state_machine LINK ${GRVCQUADROTOR_LIBRARIES})
