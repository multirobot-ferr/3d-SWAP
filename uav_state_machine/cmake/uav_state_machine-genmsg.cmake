# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(FATAL_ERROR "Could not find messages which '/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/waypoint_service.srv' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [PoseStamped]: unknown package [geometry_msgs] on search path [{'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}]")
message(STATUS "uav_state_machine: 0 messages, 4 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uav_state_machine_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv" NAME_WE)
add_custom_target(_uav_state_machine_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uav_state_machine" "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv" ""
)

get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv" NAME_WE)
add_custom_target(_uav_state_machine_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uav_state_machine" "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv" ""
)

get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv" NAME_WE)
add_custom_target(_uav_state_machine_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uav_state_machine" "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uav_state_machine
)
_generate_srv_cpp(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uav_state_machine
)
_generate_srv_cpp(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uav_state_machine
)

### Generating Module File
_generate_module_cpp(uav_state_machine
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uav_state_machine
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uav_state_machine_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uav_state_machine_generate_messages uav_state_machine_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_cpp _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_cpp _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_cpp _uav_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uav_state_machine_gencpp)
add_dependencies(uav_state_machine_gencpp uav_state_machine_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uav_state_machine_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uav_state_machine
)
_generate_srv_eus(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uav_state_machine
)
_generate_srv_eus(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uav_state_machine
)

### Generating Module File
_generate_module_eus(uav_state_machine
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uav_state_machine
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uav_state_machine_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uav_state_machine_generate_messages uav_state_machine_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_eus _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_eus _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_eus _uav_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uav_state_machine_geneus)
add_dependencies(uav_state_machine_geneus uav_state_machine_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uav_state_machine_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uav_state_machine
)
_generate_srv_lisp(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uav_state_machine
)
_generate_srv_lisp(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uav_state_machine
)

### Generating Module File
_generate_module_lisp(uav_state_machine
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uav_state_machine
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uav_state_machine_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uav_state_machine_generate_messages uav_state_machine_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_lisp _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_lisp _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_lisp _uav_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uav_state_machine_genlisp)
add_dependencies(uav_state_machine_genlisp uav_state_machine_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uav_state_machine_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uav_state_machine
)
_generate_srv_nodejs(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uav_state_machine
)
_generate_srv_nodejs(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uav_state_machine
)

### Generating Module File
_generate_module_nodejs(uav_state_machine
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uav_state_machine
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(uav_state_machine_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(uav_state_machine_generate_messages uav_state_machine_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_nodejs _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_nodejs _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_nodejs _uav_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uav_state_machine_gennodejs)
add_dependencies(uav_state_machine_gennodejs uav_state_machine_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uav_state_machine_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uav_state_machine
)
_generate_srv_py(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uav_state_machine
)
_generate_srv_py(uav_state_machine
  "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uav_state_machine
)

### Generating Module File
_generate_module_py(uav_state_machine
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uav_state_machine
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uav_state_machine_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uav_state_machine_generate_messages uav_state_machine_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/land_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_py _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/takeoff_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_py _uav_state_machine_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luisgrvc/catkin_ws/src/mbzirc/uav_state_machine/srv/target_service.srv" NAME_WE)
add_dependencies(uav_state_machine_generate_messages_py _uav_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uav_state_machine_genpy)
add_dependencies(uav_state_machine_genpy uav_state_machine_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uav_state_machine_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uav_state_machine)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uav_state_machine
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(uav_state_machine_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uav_state_machine)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uav_state_machine
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(uav_state_machine_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uav_state_machine)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uav_state_machine
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(uav_state_machine_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uav_state_machine)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uav_state_machine
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(uav_state_machine_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uav_state_machine)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uav_state_machine\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uav_state_machine
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(uav_state_machine_generate_messages_py std_msgs_generate_messages_py)
endif()
