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
cmake_minimum_required (VERSION 2.6)
project(mbzirc)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ---- Configurable options ----
# Pass this options to CMake in command line to select specific functionality for this project
option(MBZIRC_BUILD_TESTS "Enable testing and generate test projects" ON)
option(MBZIRC_BUILD_SAMPLES "Build sample projects" ON)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	option(MBZIRC_USE_ROS "Enable ros-specific code and interfaces" ON)
else()
	option(MBZIRC_USE_ROS "Enable ros-specific code and interfaces" OFF)
endif()

# --- Compiler specific options ---
# Visual studio specifics
if(MSVC)
	add_definitions(/W4 /WX) # Enable high level warnings and treat them like errors
endif(MSVC)
if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-std=c++11 -Wall -Werror -DNOMINMAX)
endif()

if(MBZIRC_USE_ROS)
	add_definitions(-DGRVC_USE_ROS)
endif(MBZIRC_USE_ROS)

#find_package(Doxygen)
#if(DOXYGEN_FOUND)
#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
#add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#	COMMENT "Generating API documentation with Doxygen" VERBATIM)
#endif(DOXYGEN_FOUND)

# Included subprojects
add_subdirectory(gcs_core)

if(MBZIRC_BUILD_TESTS)
	ENABLE_TESTING()
endif(MBZIRC_BUILD_TESTS)