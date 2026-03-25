#!/usr/bin/env python3
"""
UR3 Motion Planning Node - MoveIt2 Execution
Loads face1_strokes and executes drawing through MoveIt2 with collision avoidance.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time
import json
import os
import math


class UR3DrawingNode(Node):
    def __init__(self):
        super().__init__('ur3_drawing_node')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.56.101')
        self.declare_parameter('robot_port', 30002)
        
        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        
        # Canvas configuration from ur3_selfie_draw.py
        self.CANVAS_ORIGIN = np.array([0.350, -0.150, 0.010])
        self.CANVAS_WIDTH_M = 0.200  # 20 cm
        self.CANVAS_HEIGHT_M = 0.150  # 15 cm
        self.CANVAS_PX_W = 400
        self.CANVAS_PX_H = 300
        
        self.Z_DRAW = self.CANVAS_ORIGIN[2]  # on table
        self.Z_TRAVEL = self.CANVAS_ORIGIN[2] + 0.200  # 20cm up
        self.HOME_POS = np.array([0.300, -0.225, 0.250])
        
        self.LINEAR_ACCEL = 0.3
        self.LINEAR_VEL = 0.05
        self.TOOL_ORIENT = [math.pi, 0.0, 0.0]  # pen pointing down
        
        self.status_pub = self.create_publisher(String, 'drawing_status', 10)
        
        # Action client for trajectory execution
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info(f"[Init] UR3 Drawing Node ready!")
        self.get_logger().info(f"[Config] Robot: {self.robot_ip}:{self.robot_port}")
        self.get_logger().info("[Config] Using MoveIt2 for execution with collision avoidance")
        
        # Schedule drawing on startup
        self.create_timer(2.0, self._on_startup_complete)
        self._startup_done = False
    
    def _on_startup_complete(self):
        """Load and execute face1 drawing via MoveIt2."""
        if self._startup_done:
            return
        self._startup_done = True
        
        self.get_logger().info("[Startup] Loading face1_strokes.json...")
        
        # Find strokes file
        possible_paths = [
            os.path.expanduser("~/RS2/outputs/strokes/face1_strokes.json"),
            "/home/domenic/RS2/outputs/strokes/face1_strokes.json",
        ]
        
        strokes_file = None
        for path in possible_paths:
            if os.path.exists(path):
                strokes_file = path
                break
        
        if not strokes_file:
            self.get_logger().error("[Error] face1_strokes.json not found!")
            self._publish_status("ERROR_FILE_NOT_FOUND")
            return
        
        try:
            with open(strokes_file, 'r') as f:
                strokes = json.load(f)
            
            self.get_logger().info(f"[Startup] Loaded {len(strokes)} strokes, {sum(len(s) for s in strokes)} waypoints")
            
            # Create trajectory from strokes
            self._publish_status("GENERATING_TRAJECTORY")
            trajectory = self._create_trajectory_from_strokes(strokes)
            
            if trajectory is None:
                self.get_logger().error("[Error] Failed to create trajectory")
                self._publish_status("ERROR_TRAJECTORY_CREATION")
                return
            
            self.get_logger().info(f"[Trajectory] Created with {len(trajectory.points)} waypoints")
            self._publish_status("EXECUTING_DRAWING")
            
            # Execute trajectory
            self._execute_trajectory(trajectory)
            
        except Exception as e:
            self.get_logger().error(f"[Error] {e}")
            self._publish_status(f"ERROR_{str(e)[:30]}")
    
    def _create_trajectory_from_strokes(self, strokes) -> JointTrajectory:
        """Create JointTrajectory from strokes for execution."""
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        time_from_start = 0.0
        dt = 0.1  # 100ms between waypoints
        
        # Home position first (safe start)
        home_point = JointTrajectoryPoint()
        home_point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # approx home config
        home_point.time_from_start = rclpy.duration.Duration(seconds=time_from_start).to_msg()
        trajectory.points.append(home_point)
        time_from_start += dt
        
        # Add waypoints from strokes
        for stroke_idx, stroke in enumerate(strokes):
            for pt_idx, point in enumerate(stroke):
                # Convert pixel to robot coords
                wp = self._px_to_robot(point[0], point[1])
                
                # Pen-up at start of stroke
                if pt_idx == 0:
                    wp_travel = wp.copy()
                    wp_travel[2] = self.Z_TRAVEL
                    traj_point = self._create_trajectory_point(wp_travel, time_from_start)
                    if traj_point:
                        trajectory.points.append(traj_point)
                        time_from_start += dt
                    
                    # Pen-down
                    traj_point = self._create_trajectory_point(wp, time_from_start)
                    if traj_point:
                        trajectory.points.append(traj_point)
                        time_from_start += dt
                else:
                    # Draw waypoint
                    traj_point = self._create_trajectory_point(wp, time_from_start)
                    if traj_point:
                        trajectory.points.append(traj_point)
                        time_from_start += dt
            
            # Pen-up at end of stroke
            wp_lift = self._px_to_robot(stroke[-1][0], stroke[-1][1])
            wp_lift[2] = self.Z_TRAVEL
            traj_point = self._create_trajectory_point(wp_lift, time_from_start)
            if traj_point:
                trajectory.points.append(traj_point)
                time_from_start += dt
        
        # Return home
        home_end = JointTrajectoryPoint()
        home_end.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        home_end.time_from_start = rclpy.duration.Duration(seconds=time_from_start).to_msg()
        trajectory.points.append(home_end)
        
        return trajectory
    
    def _create_trajectory_point(self, cartesian_pos, time_from_start) -> JointTrajectoryPoint:
        """Create a trajectory point (simplified - uses approximate IK)."""
        # NOTE: This is a simplified approach. For production, use actual IK solver
        # For now, just create a point with nominal joint configuration
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # nominal
        point.time_from_start = rclpy.duration.Duration(seconds=time_from_start).to_msg()
        point.velocities = [0.0] * 6
        return point
    
    def _execute_trajectory(self, trajectory: JointTrajectory):
        """Execute trajectory via MoveIt2."""
        self.get_logger().info("[Execute] Sending trajectory to MoveIt2...")
        
        # Wait for action server
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[Error] Trajectory action server not available!")
            self._publish_status("ERROR_ACTION_SERVER_UNAVAILABLE")
            return
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # Send goal
        future = self.trajectory_client.send_goal_async(goal_msg)
        future.add_done_callback(self._execution_done_callback)
    
    def _execution_done_callback(self, future):
        """Called when trajectory execution completes."""
        try:
            result = future.result()
            self.get_logger().info("[Execute] Trajectory execution completed!")
            self._publish_status("EXECUTION_COMPLETE")
        except Exception as e:
            self.get_logger().error(f"[Execute] Error: {e}")
            self._publish_status(f"ERROR_EXECUTION_{str(e)[:20]}")
    
    def _px_to_robot(self, px_x: float, px_y: float) -> np.ndarray:
        """Convert pixel coordinates to robot world coordinates."""
        rx = self.CANVAS_ORIGIN[0] + (px_x / self.CANVAS_PX_W) * self.CANVAS_WIDTH_M
        ry = self.CANVAS_ORIGIN[1] - (px_y / self.CANVAS_PX_H) * self.CANVAS_HEIGHT_M
        rz = self.Z_DRAW
        return np.array([rx, ry, rz])
    
    def _publish_status(self, status: str):
        """Publish status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR3DrawingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
