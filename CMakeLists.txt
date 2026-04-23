cmake_minimum_required(VERSION 3.5)
project(mower_gazebo)

option(IOT_COMM "Enable IOT transform" OFF)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(IOT_COMM)
  add_definitions(-DTOPIC_PREFIX=\"iot_\")
else()
  add_definitions(-DTOPIC_PREFIX=\"\")
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mower_msgs REQUIRED)
find_package(mower_wrapper_client REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(gazebo_simulator_node src/gazebo_simulator_node.cpp)
target_include_directories(gazebo_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${mower_wrapper_client_INCLUDE_DIRS}
)
ament_target_dependencies(gazebo_simulator_node
  gazebo_msgs
  geometry_msgs
  mower_msgs
  mower_wrapper_client
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  gazebo_simulator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
