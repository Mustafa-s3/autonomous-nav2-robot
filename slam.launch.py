import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration 

def generate_launch_description():
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    # slam_toolbox_pkg = get_package_share_directory('slam_toolbox') # Commented out for static nav
    xacro_file = "/home/ubuntu/hariri_ws/src/robot_description/urdf/robot.urdf.xacro"

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_yaml_file = LaunchConfiguration('map', default='/home/ubuntu/my_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        nav2_bringup_dir, 'params', 'nav2_params.yaml'))
    
    # Use xacro command to generate XML
    robot_description_content = Command(['xacro ', xacro_file])
        
    # Create the parameters dictionary
    params = {'robot_description': robot_description_content}

    # Defining the Nav2 bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        # LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_pkg, 'launch', 'sllidar_c1_launch.py')
            )
        ),

        # Motor controller node
        Node(
            package='motor_controller',
            executable='motor_node',
            name='motor_controller',
            output='screen'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),

        # Nav2 bringup
        nav2_bringup_launch
    ])
