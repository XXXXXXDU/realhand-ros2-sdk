#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'real_hand_ros2_sdk'

this_dir = os.path.abspath(os.path.dirname(__file__))
custom_dir = os.path.join(this_dir, package_name, "RealHand")

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

# for root, dirs, files in os.walk(custom_dir):
#     if files:
#         relative_path = os.path.relpath(root, os.path.join(this_dir, package_name))
#         target_path = os.path.join('share', package_name, relative_path)
#         # Fix here: the path must be a relative path
#         files_full_path = [os.path.relpath(os.path.join(root, f), start=os.getcwd()) for f in files]
#         data_files.append((target_path, files_full_path))
        

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='real-robot',
    maintainer_email='real-robot@todo.todo',
    description='ROS2 SDK for Real Hand',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'real_hand_sdk = real_hand_ros2_sdk.real_hand:main',
        ],
    },
)
