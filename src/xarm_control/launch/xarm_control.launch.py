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


    xarm_control =  Node(
                            package='xarm_control',
                            namespace='',
                            executable='xarm_moveit',
                            name='xarm_moveit',
                            output='log'
                        )
            
    data_collection =   Node(
                            package='xarm_control',
                            namespace = '',
                            executable = 'image_saver_node',
                            name='image_saver_node',
                            output='screen'
                        )

    return LaunchDescription([
        # Node(
        #     package='xarm_control',
        #     namespace='',
        #     executable='run_xarm',
        #     name='xarm_control'
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('xarm_api'),
        #         'launch'),
        #         '/lite6_driver.launch.py']),
        #         launch_arguments = { 'robot_ip' : '192.168.42.150' }.items(),
        # ),
        xarm_control,
        data_collection
    ])