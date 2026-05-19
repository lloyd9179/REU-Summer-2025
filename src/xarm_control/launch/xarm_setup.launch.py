#!/usr/bin/env python

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    print("################ Debug: xarm_setup.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('xarm_planner'),
                'launch'),
                '/lite6_planner_realmove.launch.py']),
                launch_arguments = { 'robot_ip' : '192.168.42.150' }.items(),
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', '_robot_planner.launch.py'])),
        launch_arguments={
            'dof': '6',
            'robot_type': 'lite',
        }.items(),
        )
    ])
