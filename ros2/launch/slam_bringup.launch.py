"""
ROS2 launch file for Webots SLAM
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_webots = get_package_share_directory('webots_ros2_driver')
    
    world_file = os.path.join(os.getcwd(), 'worlds', 'slam_arena.wbt')
    
    webots_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, 'launch', 'robot_launch.py')
        ),
        launch_arguments={'world': world_file, 'ros2_supervisor': 'true'}.items()
    )
    
    return LaunchDescription([webots_driver])
