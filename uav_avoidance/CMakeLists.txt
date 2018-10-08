cmake_minimum_required(VERSION 2.8.3)
project(uav_avoidance)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin 
    REQUIRED 
        dynamic_reconfigure
    COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        tf2
        uav_abstraction_layer
        visualization_msgs
        pcl_ros
        sensor_msgs
)

find_package(Armadillo REQUIRED)

generate_dynamic_reconfigure_options(
    config/pid.cfg
)



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
  LIBRARIES swap_3d_library
  CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        tf2
        uav_abstraction_layer
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
add_library(swap_3d_library
    src/core/swap.cpp
    src/core/polarobstaclediagram.cpp
)




target_link_libraries(swap_3d_library ${ARMADILLO_LIBRARIES})



# Creating the ROS node for SWAP
add_executable(3d_swap src/3d_swap.cpp)
add_dependencies(3d_swap swap_3d_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(3d_swap swap_3d_library ${catkin_LIBRARIES})

# Creating a node to test the system
add_executable(state_machine src/state_machine_test.cpp)
add_dependencies(state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(state_machine swap_3d_library ${catkin_LIBRARIES})


