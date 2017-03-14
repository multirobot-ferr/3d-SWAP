cmake_minimum_required(VERSION 3.0)
project(mbzirc_qt_gcs)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON) # Enable Visual Studio folders

# Clasify sources according to folder structure
# This macro is derived from http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir dirLabel)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child} ${dirLabel}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${dirLabel})
			source_group(${groupname} FILES
				${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

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

include("./cmake/CMakeFindDependencyMacro.cmake")

# ----- Dependencies -----
include_directories(include) # So that generated include files can be found


if(GRVCQUAD_USE_ROS)
	add_definitions(-DGRVC_USE_ROS)

	find_package(catkin REQUIRED COMPONENTS
			geometry_msgs
			mavros
			mavros_msgs
		    	roscpp
			grvc_quadrotor_uav
			uav_state_machine
			)

	catkin_package(CATKIN_DEPENDS
			grvc_quadrotor_uav
			uav_state_machine)

	include_directories(${catkin_INCLUDE_DIRS})
	link_libraries(${catkin_LIBRARIES})
endif(GRVCQUAD_USE_ROS)

find_package(Qt5Widgets)
find_package(Marble)

if(${Marble_FOUND})
	file(GLOB_RECURSE GCS_UI_FILES "ui/*.ui")
	file(GLOB_RECURSE GCS_SRC_FILES "src/*.cpp" "include/qt_gcs/*.h")
	GroupSources(ui qt_gcs)
	GroupSources(src qt_gcs)
	GroupSources("include/qt_gcs" qt_gcs)

	QT5_WRAP_UI(UI_HEADERS ${GCS_UI_FILES})
	add_executable(qt_gcs ${GCS_SRC_FILES} ${GCS_UI_FILES})
	target_link_libraries(qt_gcs Qt5::Widgets Qt5::WebKitWidgets Marble)
endif(${Marble_FOUND})
