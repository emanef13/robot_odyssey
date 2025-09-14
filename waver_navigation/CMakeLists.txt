cmake_minimum_required(VERSION 3.8)
project(waver_navigation)

# 1) Find ament_cmake rather than catkin
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# If you need any specific ROS 2 / Gazebo dependencies:
# find_package(gazebo_ros REQUIRED)
# find_package(xacro REQUIRED)
# etc.

# 2) Install any resource directories (launch files, worlds, etc.) into share/
install(
  DIRECTORY launch maps params rviz
  DESTINATION share/${PROJECT_NAME}
)


# 3) Mark this as an ament package
ament_package()
