#!/usr/bin/env python3
"""
Health Monitor Launch File
Launches health analyzer and alert manager nodes with configuration
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('health_monitor'),
        'config',
        'health_monitor_config.yaml'
    )

    return LaunchDescription([
        
        # CAN Status Node (provides robot data)
        # Note: Commented out as can_ros2_status_node is not available in this environment
        # Node(
        #     package='ros2can',
        #     executable='can_ros2_status_node',
        #     name='can_status',
        #     output='screen'
        # ),

        # Health Analyzer Node with config
        Node(
            package='health_monitor',
            executable='health_analyzer_node',
            name='health_analyzer',
            output='screen',
            parameters=[config_file]
        ),

        # Alert Manager Node
        Node(
            package='health_monitor',
            executable='alert_manager_node',
            name='alert_manager',
            output='screen'
        )
    ])