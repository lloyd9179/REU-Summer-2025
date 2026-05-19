from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    params_file_front = os.path.join(
        get_package_share_directory('usb_cam'),  # Replace with your package
        'config',  # Directory where the YAML file is located
        'params_1.yaml'  # The YAML file name
    )

    params_file_kiyo = os.path.join(
        get_package_share_directory('usb_cam'),  # Replace with your package
        'config',  # Directory where the YAML file is located
        'params_2.yaml'  # The YAML file name
    )

    # params_file_realsense = os.path.join(
    #     get_package_share_directory('usb_cam'),  # Replace with your package
    #     'config',  # Directory where the YAML file is located
    #     'params_3.yaml'  # The YAML file name
    # )

    pi_launch_path = os.path.join(
        get_package_share_directory('pi_comm_pkg'), 
        'launch',
        'pi_comm.launch.py' 
    )

    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 
        'launch',
        'rs_launch.py' 
    )

    xarm_setup_launch_path = os.path.join(
        get_package_share_directory('xarm_control'), 
        'launch',
        'xarm_setup.launch.py' 
    )

    xarm_control_launch_path = os.path.join(
        get_package_share_directory('xarm_control'), 
        'launch',
        'xarm_control.launch.py' 
    )


    usb_cam_node_front = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_cam_node',
        output="log",
        respawn=True,
        parameters=[params_file_front],
        remappings=[
            ('image_raw', '/front_cam/image_raw'),
            ('camera_info', '/front_cam/camera_info'),
            ('image_raw/compressed','/front_cam/image_compressed'),
            ('image_raw/compressedDepth','/front_cam/compressedDepth'),
            ('image_raw/theora','/front_cam/image_raw/theora')
        ],
    )


    usb_cam_node_kiyo = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='kiyo_cam_node',
        output="log",
        respawn=True,
        parameters=[params_file_kiyo],
        remappings=[
            ('image_raw', '/kiyo_cam/image_raw'),
            ('camera_info', '/kiyo_cam/camera_info'),
            ('image_raw/compressed','/kiyo_cam/image_compressed'),
            ('image_raw/compressedDepth','/kiyo_cam/compressedDepth'),
            ('image_raw/theora','/kiyo_cam/image_raw/theora')
        ],

    )

    realsense_node = Node(
        package='realsense2_camera',
        namespace='hunter',
        executable='realsense2_camera_node',
        name='realsense_cam_node',
        output="log",
        respawn=True,
        # parameters=[params_file_realsense],
        # remappings=[
        #     ('image_raw', '/realsense_cam/image_raw'),
        #     ('camera_info', '/realsense_cam/camera_info'),
        #     ('image_raw/compressed','/realsense_cam/image_compressed'),
        #     ('image_raw/compressedDepth','/realsense_cam/compressedDepth'),
        #     ('image_raw/theora','/realsense_cam/image_raw/theora')
        # ],

    )
    

    vision_perception_node = Node(
        package='vision_perception',
        namespace='hunter',
        executable='keypoint_inference.py',
        name='vision_perception',
        output="log",
    )

    waypoint_gen_node = Node(
        package='waypoint_generator',
        namespace='hunter',
        executable='reference_path_publisher.py',
        name='waypoint_generator_node',
        output="log",
        )


    terra_mpc_node = Node(
        package='terra_mpc',
        namespace='hunter',
        executable='mpc_ackermann.py',
        name='mpc_controller_node',
        output="log",
        respawn = True
    )


    state_machine_node = Node(
        package='state_machine_pkg',
        executable='state_machine.py',
        name='state_Machine_node',
        output="screen",
    )

    hunter_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hunter_base'),
                'launch',
                'hunter_base.launch.py'
            )
        )
    )




    # include another launch file
    # rosbag_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('vision-nav-manager'),
    #             'launch/rosbag_manager.launch.xml'))
    # )


    pi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pi_launch_path)
        # # Optional: Pass arguments to the included launch file if needed
        # launch_arguments={
        #     'arg_name': 'arg_value'  # Example of passing arguments
        # }.items()
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path)
        # # Optional: Pass arguments to the included launch file if needed
        # launch_arguments={
        #     'arg_name': 'arg_value'  # Example of passing arguments
        # }.items()
    )
    xarm_setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xarm_setup_launch_path)
    )

    xarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xarm_control_launch_path)
    )
    return LaunchDescription([
            hunter_base_launch,
            usb_cam_node_front,
            usb_cam_node_kiyo,
            realsense_launch,
            # realsense_node,
            vision_perception_node,
            waypoint_gen_node,
            terra_mpc_node,
            state_machine_node,
            xarm_setup_launch,
            xarm_control_launch,
            
            # rosbag_launch
            pi_launch


    ])