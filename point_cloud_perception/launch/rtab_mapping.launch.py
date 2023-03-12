import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('navigation_tb3'),'config')
    map_file = os.path.join(config_dir,'tb3_map.yaml')
    param_file = os.path.join(config_dir,'tb3_nav2_params.yaml')
    rviz_config_dir = os.path.join(config_dir,'navigation.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('point_cloud_perception'),'/launch','/3d_depth_mapping_rtab.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_gazebo'),'/launch','/turtlebot3_world.launch.py']),

        ),

    ])