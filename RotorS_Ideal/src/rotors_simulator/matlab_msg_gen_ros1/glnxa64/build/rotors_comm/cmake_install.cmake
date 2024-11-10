# Install script for directory: /home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rotors_comm/msg" TYPE FILE FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/msg/WindSpeed.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rotors_comm/srv" TYPE FILE FILES
    "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/Octomap.srv"
    "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/PerformanceMetrics.srv"
    "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/RecordRosbag.srv"
    "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/reset_simulation.srv"
    "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/start_simulation.srv"
    "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/srv/stop_simulation.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rotors_comm/cmake" TYPE FILE FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/build/rotors_comm/catkin_generated/installspace/rotors_comm-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/devel/include/rotors_comm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/lollogioddi/.matlab/R2023a/ros1/glnxa64/venv/bin/python3" -m compileall "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/devel/lib/python3/dist-packages/rotors_comm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/devel/lib/python3/dist-packages/rotors_comm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/build/rotors_comm/catkin_generated/installspace/rotors_comm.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rotors_comm/cmake" TYPE FILE FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/build/rotors_comm/catkin_generated/installspace/rotors_comm-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rotors_comm/cmake" TYPE FILE FILES
    "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/build/rotors_comm/catkin_generated/installspace/rotors_commConfig.cmake"
    "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/build/rotors_comm/catkin_generated/installspace/rotors_commConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rotors_comm" TYPE FILE FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/devel/lib/librotors_comm_matlab.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librotors_comm_matlab.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librotors_comm_matlab.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librotors_comm_matlab.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/m/" TYPE DIRECTORY FILES "/home/lollogioddi/catkin_ws/src/rotors_simulator/matlab_msg_gen_ros1/glnxa64/src/rotors_comm/m/" FILES_MATCHING REGEX "/[^/]*\\.m$")
endif()

