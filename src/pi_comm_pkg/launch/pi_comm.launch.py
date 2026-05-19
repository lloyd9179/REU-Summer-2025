#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='pi_comm_pkg',
        #    namespace='',
        #    executable='pi_comm_pkg_exec',
        #    name='pi_comm_node'
        #),
        Node(
	    package='pi_comm_pkg',
	    namespace='',
	    executable='pan_tilt_pub',
	    name='pan_tilt_pub_node'
	    ),
        # Node(
        #     package='pi_comm_pkg',
        #     namespace='',
        #     executable='pan_tilt_test',
        #     name='pan_tilt_test_node'
        # )

        Node(
                package='pi_comm_pkg',
                namespace = '',
                executable = 'realsense_image_saver_node',
                name='realsense_image_saver_node',
                output='screen'
            )
        
    ])
