#!/usr/bin/env python3
"""
UR3 Motion Planning Node - Using ROS 2 Services (No moveit_commander)
Controls MoveIt2 planning through ROS 2 service calls.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped, Quaternion
from std_msgs.msg import String
from moveit_msgs.srv import GetPlanningScene, MotionPlanRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time
import json
import os


class UR3MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('ur3_motion_planning_node_ros_services')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.56.101')
        self.declare_parameter('robot_port', 30002)
        self.declare_parameter('use_ros_control', False)
        self.declare_parameter('optimization_enabled', True)
        
        robot_ip = self.get_parameter('robot_ip').value
        self.use_ros_control = self.get_parameter('use_ros_control').value
        
        self.get_logger().info(f"[Init] Motion Planning Node (ROS Services) ready!")
        self.get_logger().info(f"[Config] Robot IP: {robot_ip}")
        self.get_logger().info(f"[Config] ROS Control: {self.use_ros_control}")
        
        # ROS 2 subscribers/publishers/clients
        self.stroke_sub = self.create_subscription(
            PoseArray, 'stroke_paths', self.stroke_callback, 10)
        self.status_pub = self.create_publisher(String, 'planning_status', 10)
        self.script_pub = self.create_publisher(String, 'urscript_program', 10)
        
        # Service clients
        self.planning_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene')
        
        # Configuration
        self.CANVAS_ORIGIN = np.array([0.350, -0.150, 0.010])
        self.Z_DRAW = 0.010
        self.Z_TRAVEL = 0.200
        self.LINEAR_VEL = 0.08
        self.CANVAS_PX_W = 400
        self.CANVAS_PX_H = 300
        
        # Schedule loading face1 strokes after initialization
        self.create_timer(1.0, self._on_startup_complete)
        
        self.get_logger().info("[Subscribe] Listening on /stroke_paths")
        self.get_logger().info("[Info] Table already published via add_table_simple script")
    
    def _on_startup_complete(self):
        """Called after initialization to load and execute face1 strokes."""
        # Only run once
        if hasattr(self, '_startup_done'):
            return
        self._startup_done = True
        
        self.get_logger().info("[Startup] Loading face1_strokes.json...")
        
        # Find face1_strokes.json
        possible_paths = [
            os.path.expanduser("~/RS2/outputs/strokes/face1_strokes.json"),
            "/home/domenic/RS2/outputs/strokes/face1_strokes.json",
            "../../../outputs/strokes/face1_strokes.json",
        ]
        
        strokes_file = None
        for path in possible_paths:
            if os.path.exists(path):
                strokes_file = path
                break
        
        if not strokes_file:
            self.get_logger().error("[Startup] face1_strokes.json not found!")
            return
        
        try:
            with open(strokes_file, 'r') as f:
                strokes_data = json.load(f)
            
            self.get_logger().info(f"[Startup] Loaded {len(strokes_data)} strokes from face1")
            
            # Convert strokes to flat waypoint list
            waypoints = []
            for stroke in strokes_data:
                for point in stroke:
                    pose = Pose()
                    pose.position.x = float(point[0])
                    pose.position.y = float(point[1])
                    pose.position.z = 0.0
                    waypoints.append(pose)
            
            self.get_logger().info(f"[Startup] Converted to {len(waypoints)} waypoints")
            self.get_logger().info("[Startup] NOTE: Use RViz Motion Planning panel to execute (table avoidance ready)")
            self._publish_status("READY_FOR_PLANNING")
            
        except Exception as e:
            self.get_logger().error(f"[Startup] Error: {e}")
    
    def stroke_callback(self, msg: PoseArray):
        """Handle incoming stroke paths."""
        self.get_logger().info("[Subscribe] Received stroke paths via /stroke_paths")
        self.get_logger().info("[Info] Use RViz Motion Planning panel to plan/execute with table avoidance")
    
    def _publish_status(self, status: str):
        """Publish planning status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR3MotionPlanningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
