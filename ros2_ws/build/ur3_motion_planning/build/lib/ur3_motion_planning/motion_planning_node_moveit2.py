#!/usr/bin/env python3
"""
UR3 Motion Planning Node - MoveIt2 Integration
Team Picasso | Motion Planning Subsystem
Uses MoveIt2 for path planning with collision avoidance on workspace table.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from std_msgs.msg import String
import numpy as np
import time
import json
import os

class UR3MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('ur3_motion_planning_node_moveit2')
        
        # Lazy import moveit_commander to allow other modules to run
        try:
            from moveit_commander import MoveGroupCommander, PlanningSceneInterface
            from moveit_msgs.msg import CollisionObject
            from shape_msgs.msg import SolidPrimitive
        except ImportError as e:
            self.get_logger().error(f"moveit_commander not available: {e}")
            self.get_logger().error("Install with: sudo apt install ros-humble-moveit")
            return
        
        self.MoveGroupCommander = MoveGroupCommander
        self.PlanningSceneInterface = PlanningSceneInterface
        self.CollisionObject = CollisionObject
        self.SolidPrimitive = SolidPrimitive
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.56.101')
        self.declare_parameter('robot_port', 30002)
        self.declare_parameter('use_ros_control', False)
        self.declare_parameter('optimization_enabled', True)
        
        robot_ip = self.get_parameter('robot_ip').value
        self.use_ros_control = self.get_parameter('use_ros_control').value
        
        self.get_logger().info(f"[Init] Motion Planning Node (MoveIt2) ready!")
        self.get_logger().info(f"[Config] Robot IP: {robot_ip}")
        self.get_logger().info(f"[Config] ROS Control: {self.use_ros_control}")
        
        # MoveIt2 setup
        try:
            self.move_group = self.MoveGroupCommander("manipulator")
            self.planning_scene = self.PlanningSceneInterface()
            self.get_logger().info("[MoveIt2] Move group initialized")
        except Exception as e:
            self.get_logger().error(f"[MoveIt2] Initialization failed: {e}")
            return
        
        # Add table collision object to planning scene
        self._add_table_collision()
        
        # ROS 2 subscribers/publishers
        self.stroke_sub = self.create_subscription(
            PoseArray, 'stroke_paths', self.stroke_callback, 10)
        self.status_pub = self.create_publisher(String, 'planning_status', 10)
        self.script_pub = self.create_publisher(String, 'urscript_program', 10)
        
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
    
    def _add_table_collision(self):
        """Add table as collision object to prevent robot self-collision."""
        table_pose = PoseStamped()
        table_pose.header.frame_id = "world"
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.07
        
        # Table dimensions: 1.7m x 1.7m x 0.2m
        table_size = [1.7, 1.7, 0.2]  # length, width, height (meters)
        
        # Rotation: 0, 0, 0 (identity quaternion)
        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.0
        table_pose.pose.orientation.w = 1.0
        
        collision_object = CollisionObject()
        collision_object.id = "table"
        collision_object.header.frame_id = "world"
        collision_object.pose = table_pose.pose
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = table_size
        
        collision_object.primitives.append(primitive)
        collision_object.operation = CollisionObject.ADD
        
        self.planning_scene.add_object(collision_object)
        self.get_logger().info(f"[Planning Scene] Table collision object added (1.7×1.7×0.2m at z=-0.07)")
    
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
            self.get_logger().info("[Startup] Starting face1 drawing execution...")
            
            # Execute the drawing
            self._plan_and_execute_drawing(waypoints)
            
        except Exception as e:
            self.get_logger().error(f"[Startup] Error: {e}")
    
    def stroke_callback(self, msg: PoseArray):
        """Handle incoming stroke paths."""
        self.get_logger().info("[Subscribe] Received stroke paths")
        
        # Convert message to waypoint list
        waypoints = []
        for pose in msg.poses:
            waypoints.append(pose.position)
        
        if not waypoints:
            self.get_logger().warn("[Plan] No waypoints received")
            return
        
        self.get_logger().info(f"[Plan] {len(waypoints)} waypoints loaded")
        self._publish_status("PROCESSING")
        
        # Plan and execute trajectory
        self._plan_and_execute_drawing(waypoints)
    
    def _plan_and_execute_drawing(self, waypoints):
        """Plan trajectory with MoveIt2 and execute on robot."""
        try:
            # Home position
            self.get_logger().info("[Plan] Moving to home position...")
            self.move_group.set_named_target("home")
            plan = self.move_group.plan()
            
            if plan[0]:  # Check if planning succeeded
                self.move_group.execute(plan[1], wait=True)
                self.get_logger().info("[Execute] Home position reached")
            
            # Convert pixel coordinates to robot coordinates
            robot_waypoints = self._pixel_to_robot_coords(waypoints)
            
            # Plan Cartesian path for drawing (collision-aware)
            self.get_logger().info("[Plan] Planning drawing trajectory...")
            self._publish_status("OPTIMIZING")
            
            # Create waypoint list for Cartesian path
            cartesian_waypoints = []
            for wp in robot_waypoints:
                pose = PoseStamped()
                pose.header.frame_id = "base_link"
                pose.pose.position.x = float(wp[0])
                pose.pose.position.y = float(wp[1])
                pose.pose.position.z = float(wp[2])
                
                # Fixed tool orientation (pen pointing down)
                pose.pose.orientation.x = 1.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 0.0
                cartesian_waypoints.append(pose)
            
            # Plan Cartesian path
            (plan_cartesian, fraction) = self.move_group.compute_cartesian_path(
                cartesian_waypoints, 
                0.01,  # eef_step
                0.0    # jump_threshold
            )
            
            self.get_logger().info(f"[Plan] Cartesian path planning: {fraction*100:.1f}% success")
            self._publish_status("GENERATING_SCRIPT")
            
            if fraction > 0.8:  # At least 80% of path planned
                # Execute trajectory
                self.move_group.execute(plan_cartesian, wait=True)
                self.get_logger().info("[Execute] Drawing trajectory complete")
                
                # Return home
                self.move_group.set_named_target("home")
                plan = self.move_group.plan()
                if plan[0]:
                    self.move_group.execute(plan[1], wait=True)
                
                self._publish_status("SCRIPT_READY")
                self.get_logger().info("[Success] Drawing complete")
            else:
                self._publish_status("ERROR_PATH_PLANNING_FAILED")
                self.get_logger().error("[Error] Path planning failed (low success fraction)")
        
        except Exception as e:
            self.get_logger().error(f"[Error] Execution failed: {e}")
            self._publish_status(f"ERROR_{str(e)[:30]}")
    
    def _pixel_to_robot_coords(self, waypoints):
        """Convert pixel coordinates to robot world coordinates."""
        robot_waypoints = []
        
        for wp in waypoints:
            px = float(wp.x) if hasattr(wp, 'x') else float(wp[0])
            py = float(wp.y) if hasattr(wp, 'y') else float(wp[1])
            
            # Map pixel to robot coordinates
            rx = self.CANVAS_ORIGIN[0] + (px / self.CANVAS_PX_W) * 0.200
            ry = self.CANVAS_ORIGIN[1] - (py / self.CANVAS_PX_H) * 0.150
            rz = self.Z_DRAW
            
            robot_waypoints.append([rx, ry, rz])
        
        return robot_waypoints
    
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
