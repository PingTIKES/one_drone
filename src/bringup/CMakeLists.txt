cmake_minimum_required(VERSION 3.8)
project(bringup)
find_package(ament_cmake REQUIRED)

# Make CMake reconfigure when a launch/config/map/RViz/BT resource is added or
# removed. This keeps a normal `colcon build --symlink-install` sufficient for
# newly added field files instead of requiring --cmake-force-configure.
file(GLOB_RECURSE BRINGUP_RESOURCE_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/launch/*"
  "${CMAKE_CURRENT_SOURCE_DIR}/params/*"
  "${CMAKE_CURRENT_SOURCE_DIR}/map/*"
  "${CMAKE_CURRENT_SOURCE_DIR}/rviz/*"
  "${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/*"
)

install(DIRECTORY launch params map rviz behavior_trees DESTINATION share/${PROJECT_NAME})
ament_package()
