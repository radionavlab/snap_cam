cmake_minimum_required(VERSION 2.8.3)
project(snap_cam)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs message_runtime
)

include_directories(
  src
	${catkin_INCLUDE_DIRS}
  # ./libcamera/inc
  # ./libcamera/src
)

find_library(CAMERA_LIBS NAMES camera PATHS /usr/lib)

if (NOT CAMERA_LIBS)
    message(WARNING "Could not find camera libraries, not building targets that depend on it")
endif()

file(GLOB_RECURSE SOURCE_FILES "*.cc")
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${CAMERA_LIBS} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} nav_msgs_gencpp nav_msgs_generate_messages_cpp)
