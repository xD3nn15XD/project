#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  return LaunchDescription([
    DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
    
    Node(parameters=[
        get_package_share_directory("my_robot_slam") + '/config/map_params.yaml',
        {'use_sim_time': use_sim_time}
        ],
      package='slam_toolbox',
      executable='sync_slam_toolbox_node',
      name='slam_toolbox',
      output='screen'
    )
  ])