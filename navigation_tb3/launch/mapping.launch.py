import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('navigation_tb3'),'config')

    return LaunchDescription([

    Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='mapping_node',
        output='screen',
        arguments=['-configuration_directory', config_dir,'-configuration_basename','turtlebot3_lds_2d.lua']

        ),
    Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_node',
        output='screen'

        ),
    ])