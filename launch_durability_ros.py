#!/usr/bin/env python3

"""
Launch file for CNC Durability Testing System
Starts all ROS2 nodes with configured parameters
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for durability test system"""

    # Declare launch arguments
    sensor_ip_arg = DeclareLaunchArgument(
        'sensor_ip',
        default_value='192.168.2.1',
        description='ATI sensor IP address'
    )

    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='4',
        description='Camera device ID'
    )

    force_threshold_arg = DeclareLaunchArgument(
        'force_threshold',
        default_value='7.0',
        description='Force threshold for triggering image capture (Newtons)'
    )

    force_zero_arg = DeclareLaunchArgument(
        'force_zero',
        default_value='0.5',
        description='Force threshold to consider as zero (Newtons)'
    )

    command_delay_arg = DeclareLaunchArgument(
        'command_delay',
        default_value='0.25',
        description='Delay between CNC commands (seconds)'
    )

    # ATI Sensor Node
    ati_sensor_node = Node(
        package='cnc_durability_ros',
        executable='ati_sensor_node',
        name='ati_sensor_node',
        parameters=[{
            'sensor_ip': LaunchConfiguration('sensor_ip'),
            'publish_rate': 100.0
        }],
        output='screen'
    )

    # DT Image Node
    dt_image_node = Node(
        package='cnc_durability_ros',
        executable='dt_image_node',
        name='dt_image_node',
        parameters=[{
            'camera_device_id': LaunchConfiguration('camera_id'),
            'force_trigger_threshold': LaunchConfiguration('force_threshold'),
            'force_zero_threshold': LaunchConfiguration('force_zero'),
            'save_root': 'durability_test'
        }],
        output='screen'
    )

    # CNC Control Node
    cnc_control_node = Node(
        package='cnc_durability_ros',
        executable='cnc_control_node',
        name='cnc_control_node',
        parameters=[{
            'baudrate': 115200,
            'command_delay': LaunchConfiguration('command_delay'),
            'feed_rate': 500
        }],
        output='screen'
    )

    # Coordinator Node
    coordinator_node = Node(
        package='cnc_durability_ros',
        executable='durability_coordinator',
        name='durability_coordinator',
        output='screen'
    )

    return LaunchDescription([
        sensor_ip_arg,
        camera_id_arg,
        force_threshold_arg,
        force_zero_arg,
        command_delay_arg,
        ati_sensor_node,
        dt_image_node,
        cnc_control_node,
        coordinator_node
    ])
