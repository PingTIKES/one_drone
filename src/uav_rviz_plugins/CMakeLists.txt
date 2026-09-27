cmake_minimum_required(VERSION 3.8)
project(uav_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED
  src/map_odom_panel.cpp
  include/uav_rviz_plugins/map_odom_panel.hpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs pluginlib rclcpp rviz_common)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
ament_package()
