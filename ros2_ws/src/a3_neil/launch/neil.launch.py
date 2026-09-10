"""Bring up Neil's side: TF scenario publisher + grader.

Loads scenario + grader parameters from share/a3_neil/config/params.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('a3_neil'), 'config', 'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='a3_neil',
            executable='tf_scenario_node',
            name='tf_scenario_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='a3_neil',
            executable='grader',
            name='grader',
            output='screen',
            parameters=[params_file],
        ),
    ])
