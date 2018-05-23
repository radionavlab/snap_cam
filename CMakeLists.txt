cmake_minimum_required(VERSION 2.8.3)
project(snap_cam)
set(TARGET snap_cam_exec)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES snap_cam
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
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

target_link_libraries(snap_cam ${CAMERA_LIBS})

# add the ROS executables
add_executable(${TARGET} src/main.cc src/node.cc)
target_link_libraries(${TARGET} ${catkin_LIBRARIES} snap_cam)
