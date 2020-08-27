# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(WARNING "Invoking generate_messages() without having added any message or service file before.
You should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")
message(STATUS "wall_follow: 0 messages, 0 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iackermann_msgs:/opt/ros/melodic/share/ackermann_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(wall_follow_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(wall_follow
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wall_follow
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(wall_follow_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(wall_follow_generate_messages wall_follow_generate_messages_cpp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(wall_follow_gencpp)
add_dependencies(wall_follow_gencpp wall_follow_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wall_follow_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_eus(wall_follow
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wall_follow
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(wall_follow_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(wall_follow_generate_messages wall_follow_generate_messages_eus)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(wall_follow_geneus)
add_dependencies(wall_follow_geneus wall_follow_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wall_follow_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(wall_follow
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wall_follow
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(wall_follow_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(wall_follow_generate_messages wall_follow_generate_messages_lisp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(wall_follow_genlisp)
add_dependencies(wall_follow_genlisp wall_follow_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wall_follow_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_nodejs(wall_follow
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wall_follow
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(wall_follow_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(wall_follow_generate_messages wall_follow_generate_messages_nodejs)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(wall_follow_gennodejs)
add_dependencies(wall_follow_gennodejs wall_follow_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wall_follow_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(wall_follow
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wall_follow
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(wall_follow_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(wall_follow_generate_messages wall_follow_generate_messages_py)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(wall_follow_genpy)
add_dependencies(wall_follow_genpy wall_follow_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wall_follow_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wall_follow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wall_follow
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(wall_follow_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(wall_follow_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(wall_follow_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET ackermann_msgs_generate_messages_cpp)
  add_dependencies(wall_follow_generate_messages_cpp ackermann_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wall_follow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wall_follow
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(wall_follow_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(wall_follow_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(wall_follow_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET ackermann_msgs_generate_messages_eus)
  add_dependencies(wall_follow_generate_messages_eus ackermann_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wall_follow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wall_follow
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(wall_follow_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(wall_follow_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(wall_follow_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET ackermann_msgs_generate_messages_lisp)
  add_dependencies(wall_follow_generate_messages_lisp ackermann_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wall_follow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wall_follow
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(wall_follow_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(wall_follow_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(wall_follow_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET ackermann_msgs_generate_messages_nodejs)
  add_dependencies(wall_follow_generate_messages_nodejs ackermann_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wall_follow)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wall_follow\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wall_follow
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(wall_follow_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(wall_follow_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(wall_follow_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET ackermann_msgs_generate_messages_py)
  add_dependencies(wall_follow_generate_messages_py ackermann_msgs_generate_messages_py)
endif()
