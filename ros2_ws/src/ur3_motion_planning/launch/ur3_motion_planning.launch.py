from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='IP address of UR robot or Polyscope simulator'
    )
    
    declare_robot_port = DeclareLaunchArgument(
        'robot_port',
        default_value='30002',
        description='TCP port for robot connection'
    )
    
    declare_use_ros_control = DeclareLaunchArgument(
        'use_ros_control',
        default_value='false',
        description='Use ur_robot_driver (true) or direct TCP (false)'
    )
    
    declare_optimization = DeclareLaunchArgument(
        'optimization_enabled',
        default_value='true',
        description='Enable path optimization (NN + 2-Opt)'
    )
    
    # Create motion planning node using cmd (works better with ament_python console_scripts)
    motion_planning_node = Node(
        package='ur3_motion_planning',
        executable='motion_planning_node',
        name='ur3_motion_planning',
        output='screen',
        parameters=[
            {'robot_ip': LaunchConfiguration('robot_ip')},
            {'robot_port': LaunchConfiguration('robot_port')},
            {'use_ros_control': LaunchConfiguration('use_ros_control')},
            {'optimization_enabled': LaunchConfiguration('optimization_enabled')},
        ],
        remappings=[
            # You can remap topics here if needed
            # ('stroke_paths', '/perception_subsystem/stroke_paths'),
            # ('planning_status', '/gui_subsystem/robot_status'),
        ],
    )
    
    return LaunchDescription([
        declare_robot_ip,
        declare_robot_port,
        declare_use_ros_control,
        declare_optimization,
        motion_planning_node,
    ])
