cmake_minimum_required(VERSION 2.8.3)
project(snap_cam)
set(TARGET snap_cam_exec)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavlink
  mg_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES snap_cam
  CATKIN_DEPENDS mg_msgs
  DEPENDS yaml-cpp
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	# src # for mavlink
	./libcamera/inc
	./libcamera/src
)

find_library(CAMERA_LIBS NAMES camera PATHS /usr/lib)

if (NOT CAMERA_LIBS)
    message(WARNING "Could not find camera libraries, not building targets that depend on it")
endif()

## Declare a C++ library
add_library(snap_cam
  src/snapcam.cc
)

target_link_libraries(snap_cam
  ${OpenCV_LIBS}
  ${CAMERA_LIBS}
)

# add the ROS executables
add_executable(${TARGET} src/main.cc src/callback.cc src/GPS.cc)
target_link_libraries(${TARGET} yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS} snap_cam)

add_dependencies(${TARGET} mg_msgs_gencpp)
