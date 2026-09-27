cmake_minimum_required(VERSION 3.8)
project(rviz_tf_shift)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(yaml-cpp REQUIRED)

qt5_wrap_cpp(MOC_FILES
  include/tf_shift_panel.h
)

set(SRC_FILES
  src/tf_shift_panel.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${MOC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  geometry_msgs
  rclcpp
  rviz_common
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()
