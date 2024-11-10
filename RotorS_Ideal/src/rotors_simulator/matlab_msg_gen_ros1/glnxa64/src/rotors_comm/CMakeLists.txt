cmake_minimum_required(VERSION 3.9)
project(rotors_comm)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Set CMAKE_BUILD_TYPE
set(CMAKE_BUILD_TYPE Release)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(ALIAS_ROS1_CLASS_LOADER)
    add_definitions(-Dclass_loader=ros1_class_loader)
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  message_generation
  octomap_msgs
)
## System dependencies are found with CMake''s conventions
################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
    FILES
    WindSpeed.msg
)
add_service_files(
    FILES
    Octomap.srv
    PerformanceMetrics.srv
    RecordRosbag.srv
    reset_simulation.srv
    start_simulation.srv
    stop_simulation.srv
)
## Generate added messages, services and actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  octomap_msgs
  std_msgs
)
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  message_runtime
  geometry_msgs
  octomap_msgs
  roscpp
  std_msgs
  INCLUDE_DIRS include
)
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
 "/media/lollogioddi/SSD/Matlab/extern/include"
 "/media/lollogioddi/SSD/Matlab/extern/include/MatlabDataArray"
 "/media/lollogioddi/SSD/Matlab/toolbox/ros/include/ros1"
 "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/include"
 "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/include/class_loader"
 "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/console_bridge/include"
 "/media/lollogioddi/SSD/Matlab/toolbox/ros/mlroscpp/custom_messages/include"
 "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/bzip2/include"
 "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lz4/include"
 "/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/boost/include"
  "include/rotors_comm"
)
set (CMAKE_SKIP_BUILD_RPATH false)
set (CMAKE_BUILD_WITH_INSTALL_RPATH true)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
## Get list of .c files in project source directory
file(GLOB ${PROJECT_NAME}_C_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/*.c)
## Get list of .cpp files in project source directory
file(GLOB ${PROJECT_NAME}_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/*.cpp)
link_directories("/media/lollogioddi/SSD/Matlab/extern/bin/glnxa64")
link_directories("/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/console_bridge/lib")
link_directories("/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib")
link_directories("/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/boost/lib")
# Declare library
add_library(
    rotors_comm_matlab 
    SHARED
    src/rotors_comm_WindSpeed_message.cpp
    src/rotors_comm_Octomap_service.cpp
    src/rotors_comm_PerformanceMetrics_service.cpp
    src/rotors_comm_RecordRosbag_service.cpp
    src/rotors_comm_reset_simulation_service.cpp
    src/rotors_comm_start_simulation_service.cpp
    src/rotors_comm_stop_simulation_service.cpp
)
target_include_directories(rotors_comm_matlab PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#target_compile_options(rotors_comm_matlab PUBLIC $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>: -Wall -Wextra -Wpedantic>)
endif()
target_link_libraries( rotors_comm_matlab
	"/media/lollogioddi/SSD/Matlab/extern/bin/glnxa64/libMatlabDataArray.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/console_bridge/lib/libconsole_bridge.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/libros1_class_loader.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/librosconsole.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/libroscpp.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/libroscpp_serialization.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/librostime.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/librosbag_storage.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/libroslz4.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/libtopic_tools.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/libmatlab_rosbag.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/lib/libactionlib.so"
	"/media/lollogioddi/SSD/Matlab/sys/ros1/glnxa64/ros1/boost/lib/libmwboost_thread.so"
	${Boost_LIBRARIES}
	${CMAKE_DL_LIBS}
	)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(
  rotors_comm_matlab
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rotors_comm_matlab PRIVATE "ROTORS_COMM_BUILDING_LIBRARY")
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rotors_comm_matlab
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install (DIRECTORY m/ DESTINATION m/ FILES_MATCHING PATTERN "*.m")
foreach(file_i ${CUDA_BINS})
add_custom_command(
                   TARGET 
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CATKIN_DEVEL_PREFIX}/codegen/exe//${file_i}
				   )
add_custom_command(
                   TARGET 
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CATKIN_DEVEL_PREFIX}/_ert_rtw/${file_i}
				   )
endforeach(file_i)
# Generated on Mon Jun 17 17:35:18 2024
