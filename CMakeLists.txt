cmake_minimum_required(VERSION 2.8.3)
project(follow_me)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  move
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

project(DisplayImage)
set(OpenCV_DIR "/usr/local/include/opencv2/")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-pthread")

add_executable(follow_me src/follow_me.cpp include/follow_me/follow_me.h)
add_dependencies(follow_me ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(follow_me ${OpenCV_LIBS} ${catkin_LIBRARIES})
