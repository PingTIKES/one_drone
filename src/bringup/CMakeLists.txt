cmake_minimum_required(VERSION 3.8)
project(bringup)
find_package(ament_cmake REQUIRED)

# Make CMake reconfigure when an operational resource is added or removed.
file(GLOB_RECURSE BRINGUP_RESOURCE_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/launch/*"
  "${CMAKE_CURRENT_SOURCE_DIR}/params/*"
  "${CMAKE_CURRENT_SOURCE_DIR}/map/*"
  "${CMAKE_CURRENT_SOURCE_DIR}/rviz/*"
)

install(DIRECTORY launch params map rviz DESTINATION share/${PROJECT_NAME})
ament_package()
