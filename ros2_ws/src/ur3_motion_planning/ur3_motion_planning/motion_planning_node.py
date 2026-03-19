#!/usr/bin/env python3
"""
UR3 Motion Planning Node (ROS 2)
Team Picasso | Domenic Kadioglu

ROS 2 Humble node for trajectory generation and control.
Subscribes to stroke paths, publishes joint trajectories or URScript commands.
Compatible with ur_robot_driver and Polyscope simulator.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import json
import socket
import time
import numpy as np
from typing import List, Tuple, Dict, Any
import logging

# Import message types
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Point, Pose, PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Import existing motion planning functions (from sibling src directory)
import sys
import os
# Go up 4 levels: ur3_motion_planning/ur3_motion_planning/motion_planning_node.py → RS2/src/
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src'))
sys.path.insert(0, src_path)

from ur3_selfie_draw import (
    scale_strokes_to_workspace,
    nearest_neighbour_sort,
    two_opt_improve,
    build_urscript,
    validate_pose,
    px_to_robot,
    Metrics,
    ROBOT_IP,
    ROBOT_PORT,
    CANVAS_PX_W,
    CANVAS_PX_H,
    CANVAS_ORIGIN_ROBOT,
    CANVAS_WIDTH_M,
    CANVAS_HEIGHT_M,
    Z_DRAW,
    Z_TRAVEL,
    HOME_POS,
    JOINT_ACCEL,
    JOINT_VEL,
    LINEAR_ACCEL,
    LINEAR_VEL,
    TOOL_ORIENT,
)


class MotionPlanningNode(Node):
    """
    ROS 2 Node for UR3 Motion Planning.
    
    Subscribes to:
        /stroke_paths (geometry_msgs/PoseArray) - Vector paths from perception
        /motion_params (std_msgs/String) - Optional parameters (JSON)
    
    Publishes to:
        /urscript_program (std_msgs/String) - Generated URScript
        /planning_status (std_msgs/String) - Status messages
    
    Services:
        /plan_and_execute - Plan trajectory and execute on robot
    """
    
    def __init__(self):
        super().__init__('ur3_motion_planning_node')
        
        # Configure logging
        self.logger = self.get_logger()
        self.metrics = Metrics()
        
        # Parameters
        self.declare_parameter('robot_ip', ROBOT_IP)
        self.declare_parameter('robot_port', ROBOT_PORT)
        self.declare_parameter('use_ros_control', False)  # False = TCP direct, True = ur_robot_driver
        self.declare_parameter('optimization_enabled', True)
        
        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.use_ros_control = self.get_parameter('use_ros_control').value
        self.optimization_enabled = self.get_parameter('optimization_enabled').value
        
        self.logger.info(f"[Init] Robot IP: {self.robot_ip}:{self.robot_port}")
        self.logger.info(f"[Init] ROS Control: {self.use_ros_control}")
        self.logger.info(f"[Init] Optimization: {self.optimization_enabled}")
        
        # TCP socket for Polyscope/real robot
        self.robot_socket = None
        self.connected = False
        
        # Current stroke data
        self.current_strokes = []
        
        # QoS profile (matching ur_robot_driver expectations)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Create subscribers
        self.subscription_strokes = self.create_subscription(
            PoseArray,
            'stroke_paths',
            self.stroke_callback,
            qos_profile
        )
        self.logger.info("[Subscribe] Listening on /stroke_paths")
        
        self.subscription_params = self.create_subscription(
            String,
            'motion_params',
            self.params_callback,
            10
        )
        self.logger.info("[Subscribe] Listening on /motion_params")
        
        # Create publishers
        self.publisher_urscript = self.create_publisher(
            String,
            'urscript_program',
            10
        )
        self.logger.info("[Publish] Publishing to /urscript_program")
        
        self.publisher_status = self.create_publisher(
            String,
            'planning_status',
            10
        )
        self.logger.info("[Publish] Publishing to /planning_status")
        
        # Joint trajectory publisher (for ur_robot_driver if needed)
        self.publisher_trajectory = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10
        )
        self.logger.info("[Publish] Publishing to /joint_trajectory")
        
        # Try to connect to robot
        if not self.use_ros_control:
            self.connect_to_robot()
        else:
            self.logger.info("[Robot] Using ROS Control (ur_robot_driver) - connection via driver")
        
        self.logger.info("[Init] Motion Planning Node ready!")
    
    def connect_to_robot(self):
        """Connect to Polyscope simulator or real UR3 via TCP."""
        try:
            self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.robot_socket.settimeout(5.0)
            self.robot_socket.connect((self.robot_ip, self.robot_port))
            self.connected = True
            self.logger.info(f"[Robot] Connected to {self.robot_ip}:{self.robot_port}")
            self.publish_status(f"Connected to robot at {self.robot_ip}")
        except Exception as e:
            self.logger.warn(f"[Robot] Connection failed: {e}")
            self.connected = False
            self.publish_status(f"Robot connection failed: {e}")
    
    def publish_status(self, message: str):
        """Publish status message."""
        msg = String()
        msg.data = message
        self.publisher_status.publish(msg)
        self.logger.info(f"[Status] {message}")
    
    def stroke_callback(self, msg: PoseArray):
        """
        Callback when stroke paths are received from perception subsystem.
        Converts PoseArray to stroke format and triggers planning.
        """
        self.logger.info(f"[Stroke] Received {len(msg.poses)} waypoints")
        
        try:
            # Convert ROS PoseArray to stroke format (list of tuples)
            strokes = self.poses_to_strokes(msg.poses)
            self.current_strokes = strokes
            
            # Trigger motion planning
            self.plan_trajectories(strokes)
            
        except Exception as e:
            error_msg = f"Stroke processing failed: {e}"
            self.logger.error(f"[Error] {error_msg}")
            self.publish_status(error_msg)
    
    def params_callback(self, msg: String):
        """
        Callback for optional motion parameters (JSON format).
        Example: {"max_speed": 0.5, "optimization": true}
        """
        try:
            params = json.loads(msg.data)
            self.logger.info(f"[Params] Received: {params}")
            
            # Update optimization flag if specified
            if 'optimization' in params:
                self.optimization_enabled = params['optimization']
                self.logger.info(f"[Params] Optimization: {self.optimization_enabled}")
            
        except Exception as e:
            self.logger.warn(f"[Params] Invalid JSON: {e}")
    
    def poses_to_strokes(self, poses: List[Pose]) -> List[List[Tuple[float, float]]]:
        """
        Convert ROS PoseArray to stroke format.
        Assumes poses are already in pixel coordinates from perception subsystem.
        """
        if not poses or len(poses) == 0:
            return []
        
        # Group consecutive poses into strokes
        # (Perception node should mark stroke breaks somehow - for now, treat as single stroke)
        strokes = []
        current_stroke = []
        
        for pose in poses:
            x = pose.position.x
            y = pose.position.y
            current_stroke.append((x, y))
        
        if current_stroke:
            strokes.append(current_stroke)
        
        return strokes
    
    def plan_trajectories(self, strokes: List[List[Tuple[float, float]]]):
        """
        Execute motion planning pipeline:
        1. Scale strokes to workspace
        2. Optimize path ordering
        3. Generate URScript
        4. Publish/execute
        """
        self.logger.info("[Plan] Starting motion planning pipeline...")
        start_time = time.time()
        
        try:
            # Step 1: Automatic workspace scaling
            self.publish_status("Scaling strokes to workspace...")
            scaled_strokes = scale_strokes_to_workspace(strokes)
            
            # Step 2: Optimize path ordering
            if self.optimization_enabled:
                self.publish_status("Running nearest-neighbour sort...")
                optimized_strokes = nearest_neighbour_sort(scaled_strokes, self.metrics)
                
                self.publish_status("Running 2-opt refinement...")
                optimized_strokes = two_opt_improve(optimized_strokes, max_iterations=50, metrics=self.metrics)
            else:
                optimized_strokes = scaled_strokes
            
            # Step 3: Generate URScript
            self.publish_status("Generating URScript...")
            script, validation_errors = build_urscript(optimized_strokes, self.metrics)
            
            # Log validation results
            if validation_errors:
                self.logger.warn(f"[Plan] {len(validation_errors)} validation warnings")
                for err in validation_errors[:5]:  # Show first 5
                    self.logger.warn(f"  - {err}")
            else:
                self.logger.info("[Plan] All poses validated successfully")
            
            # Step 4: Publish URScript
            self.publish_urscript(script)
            
            # Step 5: Send to robot (if connected via TCP)
            if self.connected and not self.use_ros_control:
                self.send_urscript_to_robot(script)
            
            elapsed = time.time() - start_time
            self.logger.info(f"[Plan] Pipeline complete: {elapsed:.3f}s")
            self.publish_status(f"Planning complete ({elapsed:.3f}s) - Ready to execute")
            
        except Exception as e:
            error_msg = f"Planning failed: {e}"
            self.logger.error(f"[Plan] {error_msg}")
            self.publish_status(error_msg)
            raise
    
    def publish_urscript(self, script: str):
        """Publish generated URScript to topic."""
        msg = String()
        msg.data = script
        self.publisher_urscript.publish(msg)
        self.logger.info(f"[Publish] URScript ({len(script)} bytes)")
    
    def send_urscript_to_robot(self, script: str):
        """
        Send URScript to robot via TCP socket.
        This is for Polyscope simulator or direct real robot connection.
        """
        if not self.connected:
            self.logger.warn("[Robot] Not connected - script not sent")
            return
        
        try:
            payload = (script + "\n").encode("utf-8")
            self.robot_socket.sendall(payload)
            self.logger.info(f"[Robot] Script sent ({len(payload)} bytes)")
            self.publish_status(f"Script sent to robot ({len(payload)} bytes)")
            time.sleep(0.5)
        except Exception as e:
            self.logger.error(f"[Robot] Send failed: {e}")
            self.connected = False
            self.publish_status(f"Robot send error: {e}")
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        if self.robot_socket:
            try:
                self.robot_socket.close()
                self.logger.info("[Cleanup] Socket closed")
            except:
                pass
        super().destroy_node()


def main(args=None):
    """ROS 2 node entry point."""
    rclpy.init(args=args)
    node = MotionPlanningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
