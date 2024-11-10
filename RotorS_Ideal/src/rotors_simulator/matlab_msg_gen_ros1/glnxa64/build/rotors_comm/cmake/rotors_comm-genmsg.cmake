# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rotors_comm: 1 messages, 6 services")

set(MSG_I_FLAGS "-Irotors_comm:/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/msg;-Istd_msgs:/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg;-Igeometry_msgs:/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg;-Ioctomap_msgs:/media/lollogioddi/SSD/Matlab/toolbox/ros/mlroscpp/custom_messages/share/octomap_msgs/cmake/../msg;-Istd_msgs:/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg;-Iactionlib:/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg;-Iactionlib_msgs:/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rotors_comm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/msg/WindSpeed.msg" NAME_WE)
add_custom_target(_rotors_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rotors_comm" "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/msg/WindSpeed.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/Octomap.srv" NAME_WE)
add_custom_target(_rotors_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rotors_comm" "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/Octomap.srv" "std_msgs/Header:geometry_msgs/Point:octomap_msgs/Octomap"
)

get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/PerformanceMetrics.srv" NAME_WE)
add_custom_target(_rotors_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rotors_comm" "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/PerformanceMetrics.srv" ""
)

get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/RecordRosbag.srv" NAME_WE)
add_custom_target(_rotors_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rotors_comm" "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/RecordRosbag.srv" ""
)

get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/reset_simulation.srv" NAME_WE)
add_custom_target(_rotors_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rotors_comm" "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/reset_simulation.srv" ""
)

get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/start_simulation.srv" NAME_WE)
add_custom_target(_rotors_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rotors_comm" "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/start_simulation.srv" ""
)

get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/stop_simulation.srv" NAME_WE)
add_custom_target(_rotors_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rotors_comm" "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/stop_simulation.srv" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/msg/WindSpeed.msg"
  "${MSG_I_FLAGS}"
  "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg;/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
)

### Generating Services
_generate_srv_cpp(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/Octomap.srv"
  "${MSG_I_FLAGS}"
  "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg;/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/media/lollogioddi/SSD/Matlab/toolbox/ros/mlroscpp/custom_messages/share/octomap_msgs/cmake/../msg/Octomap.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
)
_generate_srv_cpp(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/PerformanceMetrics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
)
_generate_srv_cpp(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/RecordRosbag.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
)
_generate_srv_cpp(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/reset_simulation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
)
_generate_srv_cpp(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/start_simulation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
)
_generate_srv_cpp(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/stop_simulation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
)

### Generating Module File
_generate_module_cpp(rotors_comm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rotors_comm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rotors_comm_generate_messages rotors_comm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/msg/WindSpeed.msg" NAME_WE)
add_dependencies(rotors_comm_generate_messages_cpp _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/Octomap.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_cpp _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/PerformanceMetrics.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_cpp _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/RecordRosbag.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_cpp _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/reset_simulation.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_cpp _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/start_simulation.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_cpp _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/stop_simulation.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_cpp _rotors_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rotors_comm_gencpp)
add_dependencies(rotors_comm_gencpp rotors_comm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rotors_comm_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/msg/WindSpeed.msg"
  "${MSG_I_FLAGS}"
  "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg;/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
)

### Generating Services
_generate_srv_py(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/Octomap.srv"
  "${MSG_I_FLAGS}"
  "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg;/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/media/lollogioddi/SSD/Matlab/toolbox/ros/mlroscpp/custom_messages/share/octomap_msgs/cmake/../msg/Octomap.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
)
_generate_srv_py(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/PerformanceMetrics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
)
_generate_srv_py(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/RecordRosbag.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
)
_generate_srv_py(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/reset_simulation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
)
_generate_srv_py(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/start_simulation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
)
_generate_srv_py(rotors_comm
  "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/stop_simulation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
)

### Generating Module File
_generate_module_py(rotors_comm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rotors_comm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rotors_comm_generate_messages rotors_comm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/msg/WindSpeed.msg" NAME_WE)
add_dependencies(rotors_comm_generate_messages_py _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/Octomap.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_py _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/PerformanceMetrics.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_py _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/RecordRosbag.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_py _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/reset_simulation.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_py _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/start_simulation.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_py _rotors_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/stop_simulation.srv" NAME_WE)
add_dependencies(rotors_comm_generate_messages_py _rotors_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rotors_comm_genpy)
add_dependencies(rotors_comm_genpy rotors_comm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rotors_comm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rotors_comm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rotors_comm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rotors_comm_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET octomap_msgs_generate_messages_cpp)
  add_dependencies(rotors_comm_generate_messages_cpp octomap_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rotors_comm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm)
  install(CODE "execute_process(COMMAND \"/home/lollogioddi/.matlab/R2023a/ros1/glnxa64/venv/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rotors_comm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rotors_comm_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rotors_comm_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET octomap_msgs_generate_messages_py)
  add_dependencies(rotors_comm_generate_messages_py octomap_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rotors_comm_generate_messages_py std_msgs_generate_messages_py)
endif()
