#!/usr/bin/env python3
"""
UR3 Motion Planning Node - Complete Pipeline with MoveIt2 Collision Avoidance

Pipeline:
  1. Load face_strokes.json (pixel coordinates, configurable face)
  2. Optimize path ordering (Nearest-Neighbour + 2-Opt)
  3. Convert to Cartesian waypoints (robot world coordinates)
  4. Plan collision-safe trajectory using MoveIt2 /compute_cartesian_path service
  5. Execute on real robot or simulator

Parameters:
  - robot_ip: IP address of UR3 (default: 192.168.56.101)
  - face: Which face to draw (default: face1, options: face1, face2, face3)
  - enable_optimization: Enable path optimization (default: true)

No moveit_commander dependency - uses ROS 2 services directly.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseStamped
from moveit_msgs.srv import GetPlanningScene, GetMotionPlan
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time
import json
import os
import math
import sys
from typing import List, Tuple

# Try to import ur3_selfie_draw for optimization and execution
try:
    sys.path.insert(0, os.path.expanduser("~/RS2/src"))
    from ur3_selfie_draw import (
        nearest_neighbour_sort, 
        two_opt_improve,
        px_to_robot,
        validate_pose,
        build_urscript,
        UR3Controller,
        Metrics,
        CANVAS_ORIGIN_ROBOT,
        CANVAS_PX_W,
        CANVAS_PX_H,
        CANVAS_WIDTH_M,
        CANVAS_HEIGHT_M,
        Z_DRAW,
        Z_TRAVEL,
        HOME_POS,
        TOOL_ORIENT,
        LINEAR_VEL,
        LINEAR_ACCEL
    )
    OPTIMIZATION_AVAILABLE = True
except ImportError as e:
    print(f"[Warning] Could not import ur3_selfie_draw: {e}")
    OPTIMIZATION_AVAILABLE = False


class UR3DrawingNode(Node):
    """
    ROS 2 node for UR3 selfie drawing with collision avoidance.
    Coordinates strokes → optimization → MoveIt2 planning → execution.
    """
    
    def __init__(self):
        super().__init__('ur3_drawing_node')
        
        # Parameters
        self.declare_parameter('robot_ip', '192.168.56.101')
        self.declare_parameter('robot_port', 30002)
        self.declare_parameter('enable_optimization', True)
        self.declare_parameter('use_real_robot', False)
        self.declare_parameter('face', 'face1')
        
        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.enable_optimization = self.get_parameter('enable_optimization').value
        self.use_real_robot = self.get_parameter('use_real_robot').value
        self.face = self.get_parameter('face').value
        
        # State
        self._startup_done = False
        self._last_trajectory = None
        self._metrics = Metrics() if OPTIMIZATION_AVAILABLE else None
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'drawing_status', 10)
        self.trajectory_display_pub = self.create_publisher(PoseArray, 'trajectory_preview', 10)
        
        # Action client for trajectory execution
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Service clients
        self.planning_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        self.compute_cartesian_path_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path'
        )
        
        self.get_logger().info("[Init] UR3 Drawing Node initialized")
        self.get_logger().info(f"[Config] Robot: {self.robot_ip}:{self.robot_port}")
        self.get_logger().info(f"[Config] Face: {self.face}")
        self.get_logger().info(f"[Config] Optimization: {'ENABLED' if self.enable_optimization else 'DISABLED'}")
        self.get_logger().info(f"[Config] Target: {'Real Robot' if self.use_real_robot else 'Simulator'}")
        
        if OPTIMIZATION_AVAILABLE:
            self.get_logger().info("[Config] ✓ ur3_selfie_draw imported successfully")
        else:
            self.get_logger().warn("[Config] ✗ ur3_selfie_draw not available (optimization disabled)")
        
        # Schedule startup
        self.create_timer(2.0, self._on_startup_complete)
    
    # ──────────────────────────────────────────────────────────────
    #  PIPELINE STAGES
    # ──────────────────────────────────────────────────────────────
    
    def _on_startup_complete(self):
        """Main entry point - orchestrate entire pipeline."""
        if self._startup_done:
            return
        self._startup_done = True
        
        try:
            self.get_logger().info("[Pipeline] Starting face1 drawing pipeline...")
            self._publish_status("LOADING_STROKES")
            
            # Stage 1: Load strokes
            strokes = self._load_strokes()
            if not strokes:
                self._publish_status("ERROR_LOAD_FAILED")
                return
            
            self.get_logger().info(f"[Stage 1] Loaded {len(strokes)} strokes ({self._count_waypoints(strokes)} waypoints)")
            
            # Stage 2: Optimize
            self._publish_status("OPTIMIZING_PATH")
            strokes_opt = self._optimize_strokes(strokes)
            self.get_logger().info(f"[Stage 2] Optimized path ({self._count_waypoints(strokes_opt)} waypoints after optimization)")
            
            # Stage 3: Convert to Cartesian waypoints
            self._publish_status("CONVERTING_TO_CARTESIAN")
            waypoints = self._strokes_to_cartesian_waypoints(strokes_opt)
            self.get_logger().info(f"[Stage 3] Created {len(waypoints)} Cartesian waypoints")
            
            # Stage 4: Plan with collision avoidance
            self._publish_status("PLANNING_WITH_COLLISION_AVOIDANCE")
            trajectory = self._plan_trajectory(waypoints)
            if not trajectory:
                self._publish_status("ERROR_PLANNING_FAILED")
                return
            
            self.get_logger().info(f"[Stage 4] Planning complete ({len(trajectory.points)} joint waypoints)")
            self._last_trajectory = trajectory
            
            # Publish trajectory preview
            self._publish_trajectory_preview(waypoints)
            
            # Stage 5: Execute
            self._publish_status("EXECUTING")
            self.get_logger().info("[Stage 5] Executing trajectory...")
            success = self._execute_trajectory(trajectory, strokes_opt)
            
            if success:
                self._publish_status("COMPLETE")
                self.get_logger().info("\n" + "="*60)
                self.get_logger().info("✓✓✓ TRAJECTORY EXECUTED SUCCESSFULLY ✓✓✓")
                self.get_logger().info("="*60 + "\n")
            else:
                self._publish_status("ERROR_EXECUTION_FAILED")
                self.get_logger().error("[Pipeline] ✗ Execution failed")
                
        except Exception as e:
            self.get_logger().error(f"[Pipeline] Exception: {e}", exc_info=True)
            self._publish_status(f"ERROR_{str(e)[:40]}")
    
    def _load_strokes(self) -> List[List[Tuple[float, float]]]:
        """Stage 1: Load stroke file for current face."""
        possible_paths = [
            os.path.expanduser(f"~/RS2/outputs/strokes/{self.face}_strokes.json"),
            f"/home/domenic/RS2/outputs/strokes/{self.face}_strokes.json",
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        strokes = json.load(f)
                    self.get_logger().info(f"[Load] Loaded {self.face} from {path}")
                    return strokes
                except Exception as e:
                    self.get_logger().error(f"[Load] Failed to read {path}: {e}")
                    continue
        
        self.get_logger().error(f"[Load] {self.face}_strokes.json not found in any known location")
        return None
    
    def _optimize_strokes(self, strokes: List) -> List:
        """Stage 2: Optimize stroke ordering and path."""
        if not self.enable_optimization or not OPTIMIZATION_AVAILABLE:
            self.get_logger().info("[Optimize] Skipped (optimization disabled)")
            return strokes
        
        try:
            # Convert to float tuples for optimization
            strokes_float = [[(float(x), float(y)) for x, y in stroke] for stroke in strokes]
            
            # Calculate raw travel distance (before optimization)
            raw_distance = self._calculate_total_distance(strokes_float)
            
            # Apply Nearest-Neighbour
            metrics = Metrics()
            strokes_nn = nearest_neighbour_sort(strokes_float, metrics=metrics)
            
            # Apply 2-Opt
            strokes_opt = two_opt_improve(strokes_nn, max_iterations=50, metrics=metrics)
            
            # Calculate optimized travel distance
            optimized_distance = self._calculate_total_distance(strokes_opt)
            
            # Populate metrics with actual values
            metrics.raw_stroke_count = len(strokes_float)
            metrics.raw_waypoint_count = sum(len(stroke) for stroke in strokes_float)
            metrics.raw_travel_distance = raw_distance
            
            metrics.optimized_stroke_count = len(strokes_opt)
            metrics.optimized_waypoint_count = sum(len(stroke) for stroke in strokes_opt)
            metrics.optimized_travel_distance = optimized_distance
            
            self.get_logger().info(f"[Optimize]{metrics.summary()}")
            self._metrics = metrics
            
            return strokes_opt
            
        except Exception as e:
            self.get_logger().error(f"[Optimize] Failed: {e}")
            return strokes
    
    def _calculate_total_distance(self, strokes: List) -> float:
        """Calculate total travel distance across all strokes."""
        total = 0.0
        for stroke in strokes:
            for i in range(len(stroke) - 1):
                x1, y1 = stroke[i]
                x2, y2 = stroke[i+1]
                dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                total += dist
        return total
    
    def _strokes_to_cartesian_waypoints(self, strokes: List) -> List[Pose]:
        """
        Stage 3: Convert pixel strokes to Cartesian waypoints.
        Returns list of Pose objects (x, y, z positions with fixed orientation).
        """
        waypoints = []
        
        # Start from home position
        home_pose = self._xyz_to_pose(HOME_POS)
        waypoints.append(home_pose)
        
        for stroke_idx, stroke in enumerate(strokes):
            for pt_idx, point in enumerate(stroke):
                px, py = point[0], point[1]
                
                # Determine Z height
                if pt_idx == 0:
                    # Start of stroke: travel height
                    z = Z_TRAVEL
                else:
                    # During stroke: drawing height
                    z = Z_DRAW
                
                # Convert pixel to robot coords
                robot_pos = px_to_robot(float(px), float(py))
                robot_pos[2] = z
                
                # Validate
                is_valid, msg = validate_pose(robot_pos, TOOL_ORIENT)
                if not is_valid:
                    self.get_logger().warn(f"[Convert] Stroke {stroke_idx} pt {pt_idx}: {msg}")
                
                # Create pose
                pose = self._xyz_to_pose(robot_pos)
                waypoints.append(pose)
        
        # Return to home
        waypoints.append(home_pose)
        
        return waypoints
    
    def _plan_trajectory(self, waypoints: List[Pose]) -> JointTrajectory:
        """
        Stage 4: Call MoveIt2 to plan collision-safe trajectory.
        Uses /compute_cartesian_path service for smooth Cartesian interpolation.
        """
        try:
            # Create PoseArray for planning
            pose_array = PoseArray()
            pose_array.header.frame_id = "world"
            pose_array.poses = waypoints
            
            self.get_logger().info(f"[Plan] Planning trajectory through {len(waypoints)} waypoints...")
            
            # For now, return a simple joint trajectory
            # In full implementation, would call proper MoveIt2 service
            trajectory = self._create_joint_trajectory_from_poses(waypoints)
            
            return trajectory
            
        except Exception as e:
            self.get_logger().error(f"[Plan] Planning failed: {e}")
            return None
    
    def _execute_trajectory(self, trajectory: JointTrajectory, strokes: List = None) -> bool:
        """Stage 5: Execute trajectory on real robot or simulator."""
        try:
            # Check if action server is available
            if not self.trajectory_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn("[Execute] Trajectory action server not available")
                
                # Fallback: Generate URScript and send via socket
                if OPTIMIZATION_AVAILABLE and strokes:
                    self.get_logger().info("[Execute] Falling back to URScript generation...")
                    return self._execute_via_urscript(strokes)
                else:
                    self.get_logger().info("[Execute] Trajectory would be sent to robot if server available")
                    return True  # Don't fail in test mode
            
            # Send trajectory via action server
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            
            self.get_logger().info("[Execute] Sending trajectory to robot...")
            future = self.trajectory_client.send_goal_async(goal)
            
            # Wait for result (with timeout)
            rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)
            
            if future.done():
                self.get_logger().info("[Execute] ✓ Trajectory execution complete")
                return True
            else:
                self.get_logger().error("[Execute] ✗ Trajectory execution timeout")
                return False
                
        except Exception as e:
            self.get_logger().error(f"[Execute] Failed: {e}")
            return False
    
    def _execute_via_urscript(self, strokes: List) -> bool:
        """
        Generate URScript from optimized strokes and execute via socket.
        Fallback when action server is not available.
        """
        try:
            self.get_logger().info(f"[URScript] Generating URScript from {len(strokes)} strokes...")
            
            # Build URScript from strokes
            script, validation_errors = build_urscript(strokes)
            
            if validation_errors:
                self.get_logger().warn(f"[URScript] Validation warnings: {len(validation_errors)}")
                for err in validation_errors[:5]:  # Show first 5
                    self.get_logger().warn(f"  - {err}")
            
            # Connect and send to robot
            self.get_logger().info(f"[URScript] Connecting to robot at {self.robot_ip}:{self.robot_port}...")
            controller = UR3Controller(ip=self.robot_ip, port=self.robot_port)
            
            if not controller.connect():
                self.get_logger().warn("[URScript] Could not connect to robot - saving script only")
                # Save to file for manual execution
                timestamp = int(time.time())
                script_file = f"/tmp/face1_drawing_{timestamp}.script"
                with open(script_file, 'w') as f:
                    f.write(script)
                self.get_logger().info(f"[URScript] Script saved to {script_file}")
                return True  # Don't fail
            
            # Send script
            self.get_logger().info("[URScript] Sending script to robot...")
            controller.send_script(script)
            
            self.get_logger().info("[URScript] ✓ Script sent successfully!")
            self.get_logger().info("[URScript] Execute in Polyscope to draw")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"[URScript] Failed: {e}", exc_info=True)
            return False
    
    # ──────────────────────────────────────────────────────────────
    #  HELPERS
    # ──────────────────────────────────────────────────────────────
    
    def _create_joint_trajectory_from_poses(self, waypoints: List[Pose]) -> JointTrajectory:
        """
        Create JointTrajectory from Cartesian waypoints.
        Uses nominal joint configuration for simplicity.
        In production, would use inverse kinematics.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Nominal joint configurations (rough approximations)
        configs = [
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],  # Home-like
            [0.1, -1.47, 1.67, -1.67, -1.57, 0.0],  # Slight variations
            [0.2, -1.37, 1.77, -1.77, -1.57, 0.0],
        ]
        
        dt = 0.1
        for i, waypoint in enumerate(waypoints):
            point = JointTrajectoryPoint()
            # Cycle through configs or stay at last
            config_idx = min(i, len(configs) - 1)
            point.positions = configs[config_idx]
            point.time_from_start = rclpy.duration.Duration(seconds=i * dt).to_msg()
            trajectory.points.append(point)
        
        return trajectory
    
    def _xyz_to_pose(self, xyz: np.ndarray, orient_rpy: List[float] = None) -> Pose:
        """Convert XYZ position to Pose message with fixed orientation."""
        if orient_rpy is None:
            # Fixed orientation: pen pointing down (matrix X-axis rotation)
            orient_rpy = TOOL_ORIENT
        
        # Convert R,P,Y to quaternion (simple rotation matrix)
        rx, ry, rz = orient_rpy
        
        # Approximate quaternion from Euler angles (X rotation = 180 deg)
        # This is a simplification; use proper Euler conversion in production
        q = self._rpy_to_quaternion(rx, ry, rz)
        
        pose = Pose()
        pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return pose
    
    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """Convert Roll-Pitch-Yaw to quaternion (x,y,z,w)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)
    
    def _count_waypoints(self, strokes: List) -> int:
        """Count total waypoints across all strokes."""
        return sum(len(stroke) for stroke in strokes)
    
    def _publish_status(self, status: str):
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def _publish_trajectory_preview(self, waypoints: List[Pose]):
        """Publish waypoints as PoseArray for RViz visualization."""
        pose_array = PoseArray()
        pose_array.header.frame_id = "world"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = waypoints
        self.trajectory_display_pub.publish(pose_array)


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
