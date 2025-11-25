#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='real_hand_ros2_sdk',
            executable='real_hand_sdk',
            name='real_hand_sdk_left',
            output='screen',
            parameters=[{
                'hand_type': 'left', # Configure Real Hand type: left | right (lowercase)
                'hand_joint': "G20", # O6\L6P\L6\L7\L10\L20\G20 (industrial)\L21 (uppercase)
                'is_touch': True, # Configure whether Real Hand has pressure sensors: True | False
                'can': 'can0', # Change to actual CAN bus name; on Windows use something like PCAN_USBBUS1
                "modbus": "None" # "None" | "/dev/ttyUSB0" — change to actual Modbus bus name; on Windows use COM*, on Ubuntu use /dev/ttyUSB*
            }],
        ),

        Node(
            package='real_hand_ros2_sdk',
            executable='real_hand_sdk',
            name='real_hand_sdk_right',
            output='screen',
            parameters=[{
                'hand_type': 'right', # Configure Real Hand type: left | right (lowercase)
                'hand_joint': "G20", # O6\L6P\L6\L7\L10\L20\G20 (industrial)\L21 (uppercase)
                'is_touch': True, # Configure whether Real Hand has pressure sensors: True | False
                'can': 'can1', # Change to the actual CAN bus name; on Windows it might be PCAN_USBBUS1
                "modbus": "None" # "None" | "/dev/ttyUSB0" — change to the actual Modbus bus name; on Windows use COM*, on Ubuntu use /dev/ttyUSB*
            }],
        ),
    ])
