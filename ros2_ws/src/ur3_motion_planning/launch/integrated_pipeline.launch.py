#!/usr/bin/env python3
"""
Integrated launch: Perception Pipeline → Motion Planning Pipeline

Starts the selfie_perception nodes (face detection → edge extraction →
stroke mapping → visualisation) **and** the ur3_motion_planning drawing
node in "topic" mode so it subscribes to /drawing_strokes published by
the perception mapping_node.

The GUI is NOT launched here — it runs separately as a plain Python script
(~/gui/selfie_drawing_gui_ros2.py).  It publishes captured images on
/raw_image, so the image_loader_node is excluded when image_source=gui.

Usage:
    # Without GUI (perception loads from ~/perception/input/):
    ros2 launch ur3_motion_planning integrated_pipeline.launch.py

    # With GUI (start the GUI separately in another terminal):
    ros2 launch ur3_motion_planning integrated_pipeline.launch.py image_source:=gui
    python3 ~/gui/selfie_drawing_gui_ros2.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.56.101',
            description='IP address of the UR3 robot / simulator',
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='30002',
            description='URScript primary interface port',
        ),
        DeclareLaunchArgument(
            'enable_optimization',
            default_value='true',
            description='Enable NN + 2-Opt path optimisation',
        ),
        DeclareLaunchArgument(
            'display_preview',
            default_value='false',
            description='Show perception preview window (requires display)',
        ),
        DeclareLaunchArgument(
            'image_source',
            default_value='file',
            description="'file' = image_loader_node watches ~/perception/input/; "
                        "'gui' = GUI publishes on /raw_image (don't start image_loader)",
        ),
    ]

    # ── Image loader (only when not using GUI) ──
    image_loader = Node(
        package='selfie_perception',
        executable='image_loader_node',
        name='image_loader_node',
        output='screen',
        condition=LaunchConfigurationEquals('image_source', 'file'),
    )

    # ── Perception processing nodes (always started) ──
    perception_nodes = [
        Node(
            package='selfie_perception',
            executable='face_detection_node',
            name='face_detection_node',
            output='screen',
        ),
        Node(
            package='selfie_perception',
            executable='image_processing_node',
            name='image_processing_node',
            output='screen',
        ),
        Node(
            package='selfie_perception',
            executable='mapping_node',
            name='mapping_node',
            output='screen',
        ),
        Node(
            package='selfie_perception',
            executable='visualization_node',
            name='visualization_node',
            output='screen',
            parameters=[{'display': LaunchConfiguration('display_preview')}],
        ),
    ]

    # ── Motion-planning node (topic mode — waits for perception strokes) ──
    motion_node = Node(
        package='ur3_motion_planning',
        executable='motion_planning_node',
        name='ur3_drawing_node',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_port': LaunchConfiguration('robot_port'),
            'enable_optimization': LaunchConfiguration('enable_optimization'),
            'stroke_source': 'topic',
        }],
    )

    ld = LaunchDescription(declared_arguments)
    ld.add_action(image_loader)
    for n in perception_nodes:
        ld.add_action(n)
    ld.add_action(motion_node)
    return ld
