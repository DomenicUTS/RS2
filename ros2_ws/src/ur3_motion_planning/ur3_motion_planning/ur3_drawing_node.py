#!/usr/bin/env python3
"""
UR3 Motion Planning Node - Direct URScript Drawing
Loads face1_strokes.json and sends drawing commands directly to robot.
Based on ur3_selfie_draw.py pipeline.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import socket
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
        self.declare_parameter('use_socket', False)
        
        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.use_socket = self.get_parameter('use_socket').value
        
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
        
        self.get_logger().info(f"[Init] UR3 Drawing Node ready!")
        self.get_logger().info(f"[Config] Robot: {self.robot_ip}:{self.robot_port}")
        
        # Schedule drawing on startup
        self.create_timer(1.0, self._on_startup_complete)
        self._startup_done = False
    
    def _on_startup_complete(self):
        """Load and execute face1 drawing on startup."""
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
            
            # Generate URScript
            self._publish_status("GENERATING_SCRIPT")
            script = self._build_urscript(strokes)
            
            self.get_logger().info(f"[Script] Generated {len(script)} bytes from {len(strokes)} strokes")
            self._publish_status("SCRIPT_READY")
            
            # Send to robot if real hardware
            if self.use_socket:
                self.get_logger().info("[Execute] Sending to real robot...")
                self._send_to_robot(script)
            else:
                self.get_logger().info("[Execute] SIMULATION MODE - script saved")
                self._save_script(script)
            
            self._publish_status("DRAWING_COMPLETE")
            
        except Exception as e:
            self.get_logger().error(f"[Error] {e}")
            self._publish_status(f"ERROR_{str(e)[:30]}")
    
    def _px_to_robot(self, px_x: float, px_y: float) -> np.ndarray:
        """Convert pixel coordinates to robot world coordinates."""
        rx = self.CANVAS_ORIGIN[0] + (px_x / self.CANVAS_PX_W) * self.CANVAS_WIDTH_M
        ry = self.CANVAS_ORIGIN[1] - (px_y / self.CANVAS_PX_H) * self.CANVAS_HEIGHT_M
        rz = self.Z_DRAW
        return np.array([rx, ry, rz])
    
    def _pose_str(self, pos: np.ndarray, orient: list) -> str:
        """Format position and orientation for URScript.
        Returns: p[x,y,z,rx,ry,rz] format"""
        return f"p[{pos[0]:.6f},{pos[1]:.6f},{pos[2]:.6f},{orient[0]:.6f},{orient[1]:.6f},{orient[2]:.6f}]"
    
    def _build_urscript(self, strokes) -> str:
        """Generate URScript from strokes."""
        lines = []
        
        lines.append("# UR3 Selfie Drawing Robot - Face 1")
        lines.append("# Auto-generated URScript from motion planning node")
        lines.append("")
        lines.append("def draw_face():")
        lines.append(f"  # Canvas origin: {self.CANVAS_ORIGIN.tolist()}")
        lines.append(f"  # {len(strokes)} strokes loaded")
        lines.append("")
        
        # Home position
        lines.append("  # Move to safe home")
        lines.append(f"  movel({self._pose_str(self.HOME_POS, self.TOOL_ORIENT)},")
        lines.append(f"         a={self.LINEAR_ACCEL}, v={self.LINEAR_VEL})")
        lines.append("")
        
        waypoint_count = 0
        
        for s_idx, stroke in enumerate(strokes):
            if not stroke:
                continue
            
            lines.append(f"  # Stroke {s_idx + 1}")
            
            # Pen-up: move to above first point
            travel_pos = self._px_to_robot(stroke[0][0], stroke[0][1])
            travel_pos[2] = self.Z_TRAVEL
            lines.append(f"  movel({self._pose_str(travel_pos, self.TOOL_ORIENT)},")
            lines.append(f"         a={self.LINEAR_ACCEL}, v={self.LINEAR_VEL})")
            waypoint_count += 1
            
            # Pen-down: lower to first point
            draw_start = self._px_to_robot(stroke[0][0], stroke[0][1])
            lines.append(f"  movel({self._pose_str(draw_start, self.TOOL_ORIENT)},")
            lines.append(f"         a={self.LINEAR_ACCEL}, v={self.LINEAR_VEL})")
            waypoint_count += 1
            
            # Draw remaining waypoints
            for pt in stroke[1:]:
                wp = self._px_to_robot(pt[0], pt[1])
                lines.append(f"  movel({self._pose_str(wp, self.TOOL_ORIENT)},")
                lines.append(f"         a={self.LINEAR_ACCEL}, v={self.LINEAR_VEL})")
                waypoint_count += 1
            
            # Pen-up after stroke
            lift = self._px_to_robot(stroke[-1][0], stroke[-1][1])
            lift[2] = self.Z_TRAVEL
            lines.append(f"  movel({self._pose_str(lift, self.TOOL_ORIENT)},")
            lines.append(f"         a={self.LINEAR_ACCEL}, v={self.LINEAR_VEL})")
            waypoint_count += 1
            lines.append("")
        
        # Return home
        lines.append(f"  movel({self._pose_str(self.HOME_POS, self.TOOL_ORIENT)},")
        lines.append(f"         a={self.LINEAR_ACCEL}, v={self.LINEAR_VEL})")
        lines.append("end")
        lines.append("")
        lines.append("draw_face()")
        
        self.get_logger().info(f"[Script] Total waypoints: {waypoint_count}")
        
        return "\n".join(lines)
    
    def _send_to_robot(self, script: str):
        """Send URScript to real robot via socket."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.robot_ip, self.robot_port))
            self.get_logger().info(f"[Robot] Connected to {self.robot_ip}:{self.robot_port}")
            
            payload = (script + "\n").encode("utf-8")
            sock.sendall(payload)
            self.get_logger().info(f"[Robot] Script sent ({len(payload)} bytes)")
            
            sock.close()
            
        except Exception as e:
            self.get_logger().error(f"[Robot] Send failed: {e}")
    
    def _save_script(self, script: str):
        """Save URScript to file for manual execution."""
        filename = f"/tmp/face1_drawing_{int(time.time())}.script"
        with open(filename, 'w') as f:
            f.write(script)
        self.get_logger().info(f"[Save] Script saved to {filename}")
    
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
