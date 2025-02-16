import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Declare launch arguments (mirroring the old <arg> tags)
    # world_arg = DeclareLaunchArgument(
    #     'world_name',
    #     default_value=os.path.join(
    #         get_package_share_directory('waver_gazebo'),
    #         'world',
    #         'coworking.sdf'),
    #     description='Path to the world file'
    # )
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty.sdf',  # ✅ Use Gazebo's built-in empty world
        description='Path to the world file'
    )

    waver_description_path = os.path.join(
        get_package_share_directory('waver_description'))
    
    waver_robot_sim_path = os.path.join(
        get_package_share_directory('waver_gazebo'))

    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')

    # Convert xacro -> URDF at runtime
    # robot_description_content = Command([
    #     PathJoinSubstitution([FindExecutable(name='xacro')]),
    #     ' ',
    #     LaunchConfiguration('model')
    # ])

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(waver_robot_sim_path, 'worlds'), ':' +
            str(Path(waver_description_path).parent.resolve())
            ]
        )

    # Include Gazebo's standard ROS 2 launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[
            ('gz_args', LaunchConfiguration('world_name'))
            # 'paused': LaunchConfiguration('paused'),
            # # There's no direct "headless" arg in gazebo.launch.py, so you can omit or handle it differently
            # 'gui': LaunchConfiguration('gui'),
            # 'verbose': LaunchConfiguration('debug')
        ]
    )

    # (Optional) publish the robot description on /robot_description for TF, etc.
    # This is the typical approach in ROS 2: run robot_state_publisher
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'robot_description': robot_description_content
    #     }]
    # )

    xacro_file = os.path.join(waver_description_path,
                              'urdf',
                              'waver.xacro')
    
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    # (Optional) publish the robot description on /robot_description for TF, etc.
    # This is the typical approach in ROS 2: run robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    # Sprawn the robot in Gazebo by reading from /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'waver_robot',
                   '-allow_renaming', 'false'],
    )

    # Build up the entire launch description
    return LaunchDescription([
        gazebo_resource_path,
        world_arg,
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot
    ])
