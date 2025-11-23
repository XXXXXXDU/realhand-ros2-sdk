#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Suppose you have a node named "example_node" located in the real_hand_ros2_sdk package
        # and the node’s executable has already been built—typically under the src directory (confirm actual location)
        Node(
            package='matrix_touch_gui',
            executable='matrix_touch_gui',
            name='matrix_touch_gui',
            output='screen',
            parameters=[{
                'hand_type': 'right',
                'hand_joint': "L6",  # O6\L6\L7\L10\L20\L21
                'topic_hz': 30,
                'is_touch': True,
            }],
        ),
    ])
