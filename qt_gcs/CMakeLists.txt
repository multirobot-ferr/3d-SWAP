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

# ----- Dependencies -----
find_package(Qt5Widgets)
include_directories(include) # So that generated include files can be found

file(GLOB_RECURSE GCS_UI_FILES "ui/*.ui")
file(GLOB_RECURSE GCS_SRC_FILES "src/*.cpp" "include/qt_gcs/*.h")
GroupSources(ui qt_gcs)
GroupSources(src qt_gcs)
GroupSources("include/qt_gcs" qt_gcs)

QT5_WRAP_UI(UI_HEADERS ${GCS_UI_FILES})
add_executable(qt_gcs ${GCS_SRC_FILES} ${GCS_UI_FILES})
target_link_libraries(qt_gcs Qt5::Widgets)
