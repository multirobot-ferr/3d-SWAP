cmake_minimum_required(VERSION 2.8.3)
project(uav_avoidance)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
    std_msgs
    grvc_quadrotor_hal
	#grvc_utils				# TODO: REPARE
)

find_package(Armadillo REQUIRED)


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
  INCLUDE_DIRS include src/core
  LIBRARIES swap_2_5d_library
  CATKIN_DEPENDS
        roscpp
        grvc_quadrotor_hal
		#grvc_utils			# TODO: REPARE
        std_msgs
  DEPENDS ARMADILLO
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    src/core
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
)


# Declaring the C++ swap library
add_library(swap_2_5d_library
    src/core/swap.cpp
    src/core/polarobstaclediagram.cpp
	#../../collision_avoidance/swap/src/swap/core/swap.cpp
	#../../collision_avoidance/swap/src/swap/core/polarobstaclediagram.cpp
)

target_link_libraries(swap_2_5d_library ${ARMADILLO_LIBRARIES})

# Creating the ROS node for SWAP
add_executable(swap_2_5d src/swap_2_5d.cpp)
add_dependencies(swap_2_5d swap_2_5d_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(swap_2_5d swap_2_5d_library ${catkin_LIBRARIES})

# Creating a node to test the system
add_executable(state_machine src/state_machine_test.cpp)
add_dependencies(state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine swap_2_5d_library ${catkin_LIBRARIES})
