"""Bring up the professor side: TF scenario publisher + grader."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a3_professor',
            executable='tf_scenario_node',
            name='tf_scenario_node',
            output='screen',
        ),
        Node(
            package='a3_professor',
            executable='grader',
            name='grader',
            output='screen',
        ),
    ])
