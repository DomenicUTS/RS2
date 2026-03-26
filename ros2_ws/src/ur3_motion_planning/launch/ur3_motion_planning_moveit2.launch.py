#!/usr/bin/env python3
"""
Launch file for UR3 Motion Planning with MoveIt2 (All-in-One)
Starts MoveIt2 + RViz + Motion Planning Node in single command
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for UR3 MoveIt2 setup."""
    
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.56.101',
            description='IP address of UR3 robot',
        ),
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur3',
            description='UR robot type',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz for visualization',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
    ]
    
    # Include ur_moveit_config launch
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']
            )
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    
    # Add table to planning scene automatically
    add_table_node = Node(
        package='ur3_motion_planning',
        executable='python3',
        arguments=['-m', 'ur3_motion_planning.add_table_simple'],
        output='log',
        name='table_publisher',
    )
    
    ld = LaunchDescription(declared_arguments)
    ld.add_action(ur_moveit_launch)
    ld.add_action(add_table_node)
    
    return ld



