import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Get the path to the URDF/Xacro file
    default_model_path = os.path.join(
        get_package_share_directory('waver_description'),
        'urdf', 'waver.xacro'
    )

    # Get the RViz config file (if exists, otherwise remove this)
    rviz_config_path = os.path.join(
        get_package_share_directory('waver_description'),
        'rviz', 'waver.rviz'
    )

    # Declare a launch argument for the robot model
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF file'
    )

    # Process Xacro file into a robot_description parameter
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # RViz2 Node (Loads RViz with robot model)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher,
        robot_state_publisher,
        rviz_node
    ])


