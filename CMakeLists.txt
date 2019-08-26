cmake_minimum_required(VERSION 3.5)
project(follow_me)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include ./include /usr/include/eigen3)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

#project(DisplayImage)
#set(OpenCV_DIR "/usr/local/include/opencv2/")
#find_package(OpenCV REQUIRED)

#set(CMAKE_CXX_FLAGS "-pthread")

#add_compile_options(-O3 -Wall -Wextra -Wpedantic -Wunused-variable -Wunused-parameter -Wpedantic -Wdeprecated-declarations)

add_library(ExtendedKalmanFilter SHARED src/ExtendedKalmanFilter.cpp)

#add_library(PID               SHARED src/bringup/PID.cpp)
#add_library(Turtlebot         SHARED src/bringup/Turtlebot.cpp)
#ament_target_dependencies(Turtlebot rclcpp PID)

#add_executable(turtlebot2 src/bringup/turtlebot2.cpp src/bringup/PID.cpp src/bringup/Turtlebot.cpp kobuki/DockingController.cpp kobuki/Kobuki_impl.cpp kobuki/Thread.cpp kobuki/Transport.cpp kobuki/libkobuki.cpp kobuki/KobukiBase.cpp kobuki/SerialPort.cpp kobuki/Translator.cpp)

#add_executable(teleop_node src/teleop/teleop_node.cpp)

#ament_target_dependencies(turtlebot2 rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs PID lib/Turtlebot lib/DockingController lib/Kobuki_impl lib/Thread lib/Transport lib/kobuki lib/KobukiBase lib/SerialPort lib/Translator lib/kobukicwrapper)

#ament_target_dependencies(teleop_node rclcpp sensor_msgs geometry_msgs)

#target_include_directories(ExtendedKalmanFilter PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

install(TARGETS ExtendedKalmanFilter
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

#install(TARGETS teleop_node
#  EXPORT export_${PROJECT_NAME}
#  DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME})
