cmake_minimum_required(VERSION 3.5)
project(waver_description)

# Find the required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install URDF and Xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY ros2_control/
  DESTINATION share/${PROJECT_NAME}/ros2_control
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ament package definition
ament_package()