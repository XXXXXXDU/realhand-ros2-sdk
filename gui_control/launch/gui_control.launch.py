#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_control',
            executable='gui_control',
            name='left_hand_control_node',
            output='screen',
            parameters=[{
                'hand_type': 'right',   # Dexterous hand orientation: 'left' | 'right' (lowercase)
                'hand_joint': "L7",     # Hand model: O6 / L6 / L7 / L10 / L20 / G20 (industrial) / L21 (UPPERCASE)
                'topic_hz': 30,         # Topic publish frequency (Hz)
                'is_touch': True,       # Has pressure/tactile sensor
                'is_arc': False,        # Publish values in radians (True) or not (False)
            }],
        ),
        # Node(
        #     package='gui_control',
        #     executable='gui_control',
        #     name='right_hand_control_node',
        #     output='screen',
        #     parameters=[{
        #         'hand_type': 'right',
        #         'hand_joint': "L10",
        #         'topic_hz': 30,
        #         'is_touch': True,
        #     }],
        # ),
    ])
