cmake_minimum_required(VERSION 3.8)
project(uav_localization)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# GitHub's web upload stores source files without the executable bit. With
# colcon --symlink-install, installing the source directly preserves that mode
# and ROS 2 then reports that the libexec entry is missing. Copy to the build
# tree and make the installed target executable regardless of source mode.
set(VIO_BRIDGE_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/vio_to_px4.py")
configure_file(scripts/vio_to_px4.py "${VIO_BRIDGE_SCRIPT}" COPYONLY)
file(CHMOD "${VIO_BRIDGE_SCRIPT}" PERMISSIONS
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE)
install(PROGRAMS "${VIO_BRIDGE_SCRIPT}" DESTINATION lib/${PROJECT_NAME})
ament_package()
