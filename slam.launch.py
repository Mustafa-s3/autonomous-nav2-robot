from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    xacro_file = "/home/ubuntu/hariri_ws/src/robot_description/urdf/robot.urdf.xacro"
    
    # Use xacro command to generate XML
    robot_description_content = Command(['xacro ', xacro_file])
        
    # Create the parameters dictionary
    params = {'robot_description': robot_description_content}

    return LaunchDescription([
        # LiDAR (using its own launch file)
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
        
        # Node: Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        
        # SLAM Toolbox (using its online async launch file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
            )
            
        ),
    ])
