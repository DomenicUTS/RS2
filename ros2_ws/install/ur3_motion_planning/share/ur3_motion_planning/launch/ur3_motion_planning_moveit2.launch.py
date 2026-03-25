#!/usr/bin/env python3
"""
Launch file for UR3 Motion Planning with MoveIt2
Supports both RViz simulator and real robot execution
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
import os


def launch_setup(context, *args, **kwargs):
    """Dynamically configure launch based on arguments."""
    
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    
    ur_moveit_config_package = FindPackageShare('ur_moveit_config')
    
    # Motion planning node
    motion_planning_node = Node(
        package='ur3_motion_planning',
        executable='motion_planning_node_moveit2',
        name='ur3_motion_planning_node',
        output='screen',
        parameters=[
            {
                'robot_ip': robot_ip,
                'robot_port': 30002,
                'use_ros_control': False,
                'optimization_enabled': True,
                'use_sim_time': use_sim_time == 'true',
            }
        ],
    )
    
    return [motion_planning_node]


def generate_launch_description():
    """Generate launch description for UR3 MoveIt2 setup."""
    
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.56.101',
            description='IP address of UR3 robot or simulator',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (for RViz)',
        ),
    ]
    
    # MoveIt2 launch from ur_moveit_config
    ur_moveit_launch = Node(
        package='ur_moveit_config',
        executable='ur_moveit_config',
        name='ur_moveit',
        output='screen',
    )
    
    # Custom motion planning node
    motion_planning = OpaqueFunction(function=launch_setup)
    
    ld = LaunchDescription(declared_arguments)
    ld.add_action(ur_moveit_launch)
    ld.add_action(motion_planning)
    
    return ld
