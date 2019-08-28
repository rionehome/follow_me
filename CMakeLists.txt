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
find_package(OpenCV REQUIRED)

include_directories(include ./include /usr/include/eigen3 /usr/local/include/opencv2/)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

add_compile_options(-O3 -Wall -Wextra -Wpedantic -Wunused-variable -Wunused-parameter -Wpedantic -Wdeprecated-declarations)

add_library(ExtendedKalmanFilter SHARED src/ExtendedKalmanFilter.cpp)
ament_target_dependencies(ExtendedKalmanFilter)

add_executable(follow_me src/follow_me.cpp src/ExtendedKalmanFilter.cpp)
ament_target_dependencies(follow_me lib/ExtenendedKalmanFilter rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs OpenCV)

target_include_directories(follow_me PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS follow_me
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#
ament_package()
