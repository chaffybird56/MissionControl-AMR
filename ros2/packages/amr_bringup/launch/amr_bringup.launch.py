#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='amr_bringup').find('amr_bringup')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    use_webots = LaunchConfiguration('use_webots')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_share, 'maps', 'showfloor.yaml'),
        description='Full path to map yaml file to load')
    
    declare_use_webots_cmd = DeclareLaunchArgument(
        'use_webots',
        default_value='true',
        description='Whether to use Webots simulation')
    
    # Static transform publisher for robot base to laser
    static_transform_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'laser_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Static transform publisher for robot base to camera
    static_transform_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.08', '0', '0.06', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}]
    )
    
    # AMCL (localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )
    
    # Nav2 stack
    nav2_bringup_cmd = Node(
        package='nav2_bringup',
        executable='navigation_launch',
        name='navigation',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_use_webots_cmd)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(static_transform_base_to_laser)
    ld.add_action(static_transform_base_to_camera)
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(lifecycle_manager)
    ld.add_action(nav2_bringup_cmd)
    
    return ld

