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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
            'ur_type',
            default_value='ur3',
            description='UR robot type',
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
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz with MoveIt2 visualisation',
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

    # ── MoveIt2 (move_group + RViz, NO servo_node) ──
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
            'launch_servo': 'false',
            'use_sim_time': 'false',
        }.items(),
    )

    # ── Robot State Publisher (publishes URDF → /tf for RViz) ──
    ur_type = LaunchConfiguration('ur_type')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']
        ),
        ' robot_ip:=xxx.yyy.zzz.www',
        ' safety_limits:=true',
        ' safety_pos_margin:=0.15',
        ' safety_k_position:=20',
        ' name:=ur',
        ' ur_type:=', ur_type,
        ' script_filename:=ros_control.urscript',
        ' input_recipe_filename:=rtde_input_recipe.txt',
        ' output_recipe_filename:=rtde_output_recipe.txt',
        ' prefix:=',
        ' joint_limit_params:=',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'config', ur_type, 'joint_limits.yaml']
        ),
        ' kinematics_params:=',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'config', ur_type, 'default_kinematics.yaml']
        ),
        ' physical_params:=',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'config', ur_type, 'physical_parameters.yaml']
        ),
        ' visual_params:=',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'config', ur_type, 'visual_parameters.yaml']
        ),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── Joint State Publisher (provides /joint_states for TF + MoveIt2) ──
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description],
    )

    # ── Scene setup (table + marker holder) — delayed to let move_group start ──
    scene_setup = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ur3_motion_planning',
                executable='add_table',
                name='scene_setup',
                output='screen',
            ),
        ],
    )

    # ── Motion-planning node (topic mode — waits for perception strokes) ──
    #    Delayed so MoveIt2 /compute_cartesian_path service is ready.
    motion_node = TimerAction(
        period=8.0,
        actions=[
            Node(
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
            ),
        ],
    )

    ld = LaunchDescription(declared_arguments)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(ur_moveit_launch)
    ld.add_action(image_loader)
    for n in perception_nodes:
        ld.add_action(n)
    ld.add_action(scene_setup)
    ld.add_action(motion_node)
    return ld
