cmake_minimum_required(VERSION 2.8.3)
project(uav_avoidance)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp


        message_generation
        roscpp
        tf
        visualization_msgs
        std_msgs
        grvc_quadrotor_hal
        uav_state_machine
        pcl_ros
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#        DEPENDENCIES
#        std_msgs  # Or other packages containing msgs
#)

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
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
        roscpp
        grvc_quadrotor_hal
        #message_runtime
        #visualization_msgs
        #std_msgs
        #uav_state_machine
        #tf
        #DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
   #${EIGEN_INCLUDE_DIRS}
)

########################

### Finding armadillo library
set(SEARCH_LIBRARY_PATHS "/usr/lib/ /usr/local/lib/ /usr/lib32/ /usr/lib64/")

FIND_LIBRARY(LARMADILLO_LIBRARY NAMES larmadillo libarmadillo armadillo PATHS ${SEARCH_LIBRARY_PATHS})
if(NOT LARMADILLO_LIBRARY)
  message(FATAL_ERROR "armadillo library is missing:
  Please install
  $ sudo apt-get install libarmadillo-dev")
else()
  message(STATUS "armadillo library found : " ${LARMADILLO_LIBRARY})
endif()

# Declaring the C++ swap library
add_library(swap_2_5d_library
    src/core/swap.cpp
    src/core/polarobstaclediagram.cpp
)

# Creating the ROS node for SWAP
add_executable(swap_2_5d src/swap_2_5d.cpp)
add_dependencies(swap swap_2_5d_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(swap_2_5d swap_2_5d_library ${catkin_LIBRARIES} ${LARMADILLO_LIBRARY})


