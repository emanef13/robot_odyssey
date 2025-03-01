from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # Launch teleop_twist_keyboard in a new gnome-terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
            output='screen'
        ),
        Node(package = "joy",executable = "joy_node"),
        Node(package='velocity_pub', executable='robot_control.py', output='screen'),
    ])