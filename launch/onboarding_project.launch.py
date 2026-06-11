import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    walls_sensor_node = Node(
        package='onboarding_project',
        executable='walls_sensor.py',
        name='walls_sensor',
        output='screen',
    )
    walls_publisher_node = Node(
        package='onboarding_project',
        executable='rviz_hallway_publisher',
        name='rviz_hallway_publisher',
        output='screen',
    )

    ai_navigation_node = Node(
        package='onboarding_project',
        executable='AINav.py',
        name='AINav',
    )
    agent_node = Node(
        package='ros2_deepagent',
        executable='agent_node',
        name='ai_agent',
    )

    return LaunchDescription([
        walls_sensor_node,
        walls_publisher_node,
        ai_navigation_node,
        agent_node,
    ])

