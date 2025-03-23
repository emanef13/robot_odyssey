import os
from pathlib import Path
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    waver_robot_description_path = os.path.join(
        get_package_share_directory('waver_description'))
        
    waver_robot_sim_path = os.path.join(
        get_package_share_directory('waver_gazebo'))
        
    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(waver_robot_sim_path, 'worlds'), ':' +
            str(Path(waver_robot_description_path).parent.resolve())
            ]
        )
    
    rviz_config_file = os.path.join(waver_robot_description_path, 'config', 'nav.rviz')

    # Path to robot description URDF
    xacro_file = os.path.join(waver_robot_description_path,
                              'urdf',
                              'waver.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    
    # Arguments
    paused_arg = DeclareLaunchArgument('paused', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty',  # ✅ Use Gazebo's built-in empty world
        description='Path to the world file'
    )
    

    # Include Gazebo's standard ROS 2 launch file
    gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ({'gz_args': '-r camera_sensor.sdf'}, [LaunchConfiguration('world_name'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )

    ## Initializing all the nodes ## 

    # (Optional) publish the robot description on /robot_description for TF, etc.
    # This is the typical approach in ROS 2: run robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # EKF node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(waver_robot_sim_path, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn the robot in Gazebo by reading from /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '5.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'waver_robot',
                   '-allow_renaming', 'false'],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan', 
                   "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU", 
                   "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo', 
                   "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
                   "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
                   "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    #  Launching RVIZ with navigation configuration
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
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
        spawn_robot,
        bridge,
        # robot_localization_node,
        rviz,
    ])
