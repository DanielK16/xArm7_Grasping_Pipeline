cmake_minimum_required(VERSION 3.8)
project(ai_robot_control)

# Standard C++ 17 setzen
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Abhängigkeiten finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Executable erstellen
add_executable(moveit_run src/moveit_run_node.cpp)

# 3. Include Verzeichnisse
# Wir fügen /usr/include direkt hinzu, damit #include <nlohmann/json.hpp> gefunden wird
target_include_directories(moveit_run PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include
)

# 4. Abhängigkeiten verknüpfen (ROS 2 Standard)
ament_target_dependencies(moveit_run
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  std_msgs
  tf2
)

# 5. Installation
install(TARGETS moveit_run
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()