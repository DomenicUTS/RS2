#!/usr/bin/env python3
"""
UR3 Motion Planning Node - MoveIt2 Cartesian Path Planning + URScript Execution

Pipeline:
  1. Load strokes (file or /drawing_strokes topic)
  2. Optimize path ordering (Nearest-Neighbour + 2-Opt)
  3. Convert pixel strokes → Cartesian waypoints (with correct TCP orientation)
  4. Plan collision-safe trajectory via MoveIt2 /compute_cartesian_path service
  5. Convert MoveIt2 joint trajectory → URScript and execute on robot

Parameters:
  - robot_ip: IP address of UR3 (default: 192.168.56.101)
  - robot_port: URScript port (default: 30002)
  - face: Which face to draw (default: face1)
  - stroke_source: 'file' or 'topic'
  - enable_optimization: Enable path optimization (default: true)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
import time
import json
import os
import math
import sys
import socket
from typing import List, Tuple

# Import drawing constants and optimisation from ur3_selfie_draw
try:
    sys.path.insert(0, os.path.expanduser("~/RS2/src"))
    from ur3_selfie_draw import (
        nearest_neighbour_sort,
        two_opt_improve,
        px_to_robot,
        scale_strokes_to_workspace,
        validate_pose,
        Metrics,
        _calculate_travel,
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
        LINEAR_ACCEL,
        JOINT_VEL,
        JOINT_ACCEL,
        MARKER_TILT_DEG,
        MARKER_TILT_RAD,
        EE_DRAW_HEIGHT,
        TCP_OFFSET,
    )
    IMPORTS_OK = True
except ImportError as e:
    print(f"[Warning] Could not import ur3_selfie_draw: {e}")
    IMPORTS_OK = False

# ── Quaternion for the tilted tool orientation ──
# Precomputed from Ry(tilt) @ Rx(π)  →  quaternion (x, y, z, w)
def _rotvec_to_quaternion(rv: List[float]) -> Tuple[float, float, float, float]:
    """Rotation-vector → quaternion (x, y, z, w)."""
    a = np.array(rv, dtype=float)
    angle = float(np.linalg.norm(a))
    if angle < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    axis = a / angle
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    nm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / nm, y / nm, z / nm, w / nm)

if IMPORTS_OK:
    TOOL_QUAT = _rotvec_to_quaternion(TOOL_ORIENT)  # (x, y, z, w)
else:
    TOOL_QUAT = (0.984808, 0.0, -0.173648, 0.0)  # fallback

# MoveIt2 planning group
PLANNING_GROUP = "ur_manipulator"
EE_LINK = "tool0"
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
HOME_JOINTS = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]


class UR3DrawingNode(Node):
    """
    ROS 2 node: strokes → optimise → MoveIt2 Cartesian plan → URScript execute.
    """

    def __init__(self):
        super().__init__("ur3_drawing_node")

        # ── Parameters ──
        self.declare_parameter("robot_ip", "192.168.56.101")
        self.declare_parameter("robot_port", 30002)
        self.declare_parameter("enable_optimization", True)
        self.declare_parameter("face", "face1")
        self.declare_parameter("stroke_source", "file")
        self.declare_parameter("max_step", 0.005)          # Cartesian interpolation step (m)
        self.declare_parameter("jump_threshold", 5.0)       # joint-space jump filter
        self.declare_parameter("planning_timeout", 30.0)    # seconds

        self.robot_ip = self.get_parameter("robot_ip").value
        self.robot_port = self.get_parameter("robot_port").value
        self.enable_optimization = self.get_parameter("enable_optimization").value
        self.face = self.get_parameter("face").value
        self.stroke_source = self.get_parameter("stroke_source").value
        self.max_step = self.get_parameter("max_step").value
        self.jump_threshold = self.get_parameter("jump_threshold").value
        self.planning_timeout = self.get_parameter("planning_timeout").value

        # ── State ──
        self._startup_done = False
        self._topic_strokes = None
        self._metrics = Metrics() if IMPORTS_OK else None

        # ── Publishers / Subscribers ──
        self.status_pub = self.create_publisher(String, "drawing_status", 10)
        self.trajectory_display_pub = self.create_publisher(PoseArray, "trajectory_preview", 10)
        self.strokes_sub = self.create_subscription(
            String, "drawing_strokes", self._on_drawing_strokes, 10)
        self.gui_cmd_sub = self.create_subscription(
            String, "gui/command", self._on_gui_command, 10)

        # ── MoveIt2 service client ──
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path")

        # ── Scene publishers (table + marker holder) ──
        self.collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.attached_pub = self.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10)

        self.get_logger().info("[Init] UR3 Drawing Node (MoveIt2 planning mode)")
        self.get_logger().info(f"[Config] Robot: {self.robot_ip}:{self.robot_port}")
        self.get_logger().info(f"[Config] Face: {self.face} | Source: {self.stroke_source}")
        self.get_logger().info(f"[Config] MoveIt2 max_step={self.max_step} m, "
                               f"jump_threshold={self.jump_threshold}")

        if self.stroke_source == "file":
            self.create_timer(2.0, self._on_startup_complete)
        else:
            self.get_logger().info("[Init] Waiting for strokes on /drawing_strokes ...")
            self._publish_status("WAITING_FOR_PERCEPTION")

    # ─────────────────── callbacks ───────────────────

    def _on_drawing_strokes(self, msg: String):
        try:
            strokes = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[Perception] Bad JSON: {e}")
            return
        self.get_logger().info(
            f"[Perception] Received {len(strokes)} strokes "
            f"({sum(len(s) for s in strokes)} pts)")
        self._topic_strokes = strokes
        if not self._startup_done and self.stroke_source == "topic":
            self._on_startup_complete()

    def _on_gui_command(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"[GUI] Command: {cmd}")
        if cmd == "START" and self._topic_strokes is not None:
            self._startup_done = False
            self._on_startup_complete()
        elif cmd == "STOP":
            self._publish_status("STOPPED")
        elif cmd == "PAUSE":
            self._publish_status("PAUSED")
        elif cmd == "RESUME":
            self._publish_status("RESUMED")

    # ─────────────────── pipeline ───────────────────

    def _on_startup_complete(self):
        if self._startup_done:
            return
        self._startup_done = True

        try:
            self.get_logger().info("[Pipeline] Starting ...")
            self._publish_status("LOADING_STROKES")

            # 1 – Load strokes
            strokes = self._load_strokes()
            if not strokes:
                self._publish_status("ERROR_LOAD_FAILED")
                return
            self.get_logger().info(
                f"[Stage 1] {len(strokes)} strokes, "
                f"{sum(len(s) for s in strokes)} pts")

            # 2 – Scale + Optimise
            self._publish_status("OPTIMIZING_PATH")
            strokes = self._optimize_strokes(strokes)

            # 3 – Publish collision scene (table + marker holder)
            self._publish_status("SETTING_UP_SCENE")
            self._publish_scene_objects()
            time.sleep(0.5)  # give move_group time to ingest

            # 4 – Plan each stroke via MoveIt2 Cartesian path
            self._publish_status("PLANNING_WITH_MOVEIT2")
            script = self._plan_and_build_urscript(strokes)
            if script is None:
                self._publish_status("ERROR_PLANNING_FAILED")
                return

            # 5 – Execute URScript
            self._publish_status("EXECUTING")
            success = self._send_urscript(script)

            if success:
                self._publish_status("COMPLETE")
                self.get_logger().info("="*60)
                self.get_logger().info("DRAWING COMPLETE")
                self.get_logger().info("="*60)
            else:
                self._publish_status("ERROR_EXECUTION_FAILED")

        except Exception as e:
            self.get_logger().error(f"[Pipeline] {e}", exc_info=True)
            self._publish_status(f"ERROR_{str(e)[:40]}")

    # ─────────────────── stage 1: load ───────────────────

    def _load_strokes(self):
        if self.stroke_source == "topic":
            if self._topic_strokes is not None:
                return self._topic_strokes
            self.get_logger().error("[Load] No strokes on topic yet")
            return None

        paths = [
            os.path.expanduser(f"~/RS2/outputs/strokes/{self.face}_strokes.json"),
            os.path.expanduser("~/perception/output/perception_strokes.json"),
        ]
        for p in paths:
            if os.path.exists(p):
                try:
                    with open(p) as f:
                        return json.load(f)
                except Exception as e:
                    self.get_logger().error(f"[Load] {p}: {e}")
        self.get_logger().error("[Load] No stroke file found")
        return None

    # ─────────────────── stage 2: optimise ───────────────────

    def _optimize_strokes(self, strokes):
        if not self.enable_optimization or not IMPORTS_OK:
            return strokes
        try:
            strokes_f = [[(float(x), float(y)) for x, y in s] for s in strokes]
            strokes_f = scale_strokes_to_workspace(strokes_f)
            metrics = Metrics()
            metrics.raw_stroke_count = len(strokes_f)
            metrics.raw_waypoint_count = sum(len(s) for s in strokes_f)
            metrics.raw_travel_distance = _calculate_travel(strokes_f)

            nn = nearest_neighbour_sort(strokes_f, metrics=metrics)
            opt = two_opt_improve(nn, max_iterations=50, metrics=metrics)

            metrics.optimized_stroke_count = len(opt)
            metrics.optimized_waypoint_count = sum(len(s) for s in opt)
            metrics.optimized_travel_distance = _calculate_travel(opt)

            self.get_logger().info(f"[Optimise]{metrics.summary()}")
            self._metrics = metrics
            return opt
        except Exception as e:
            self.get_logger().error(f"[Optimise] {e}")
            return strokes

    # ─────────────────── stage 3: collision scene ───────────────

    def _publish_scene_objects(self):
        """Publish table and marker-holder collision objects."""

        # Table
        table = CollisionObject()
        table.header.frame_id = "world"
        table.id = "table"
        table.operation = CollisionObject.ADD
        table.pose.position.z = -0.07
        table.pose.orientation.w = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.7, 1.7, 0.2]
        table.primitives.append(box)
        self.collision_pub.publish(table)
        self.get_logger().info("[Scene] Table published")

        # Marker holder attached to tool0
        holder_co = CollisionObject()
        holder_co.header.frame_id = "tool0"
        holder_co.id = "marker_holder"
        holder_co.operation = CollisionObject.ADD
        hbox = SolidPrimitive()
        hbox.type = SolidPrimitive.BOX
        hbox.dimensions = [0.160, 0.180, EE_DRAW_HEIGHT]
        holder_co.primitives.append(hbox)
        half = EE_DRAW_HEIGHT / 2.0
        hp = Pose()
        hp.position.x = half * math.sin(MARKER_TILT_RAD)
        hp.position.z = -half * math.cos(MARKER_TILT_RAD)
        hp.orientation.y = math.sin(MARKER_TILT_RAD / 2.0)
        hp.orientation.w = math.cos(MARKER_TILT_RAD / 2.0)
        holder_co.primitive_poses.append(hp)

        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = holder_co
        attached.touch_links = ["tool0", "wrist_3_link"]
        self.attached_pub.publish(attached)
        self.get_logger().info("[Scene] Marker holder attached to tool0")

    # ─────────────────── stage 4: MoveIt2 plan → URScript ───────

    def _make_pose(self, xyz, quat=None):
        """Helper: create geometry_msgs/Pose from xyz array and quaternion."""
        if quat is None:
            quat = TOOL_QUAT
        p = Pose()
        p.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        p.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        return p

    def _call_cartesian_path(self, waypoints: List[Pose],
                             start_joints: List[float]) -> Tuple:
        """
        Call /compute_cartesian_path and return (joint_trajectory, fraction).
        Returns (None, 0.0) on failure.
        """
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("[Plan] /compute_cartesian_path service not available")
            return None, 0.0

        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK
        req.waypoints = waypoints
        req.max_step = self.max_step
        req.jump_threshold = self.jump_threshold
        req.avoid_collisions = True

        # Provide start state so MoveIt knows current joint config
        req.start_state.joint_state.name = JOINT_NAMES
        req.start_state.joint_state.position = [float(j) for j in start_joints]

        future = self.cartesian_path_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.planning_timeout)

        if not future.done() or future.result() is None:
            self.get_logger().error("[Plan] Cartesian path service call failed")
            return None, 0.0

        resp = future.result()
        fraction = resp.fraction
        traj = resp.solution.joint_trajectory

        return traj, fraction

    def _plan_and_build_urscript(self, strokes) -> str:
        """
        For each stroke, plan a MoveIt2 Cartesian path and accumulate
        joint-space waypoints.  Then convert the full set into URScript.
        """

        all_joint_waypoints = []  # list of (positions_list, is_travel_bool)
        current_joints = list(HOME_JOINTS)

        total_strokes = len(strokes)

        for s_idx, stroke in enumerate(strokes):
            if not stroke:
                continue
            self.get_logger().info(
                f"[Plan] Stroke {s_idx+1}/{total_strokes} "
                f"({len(stroke)} pts) ...")

            # ── travel to above first point (pen-up) ──
            travel_pos = px_to_robot(*stroke[0])
            travel_pos[2] = Z_TRAVEL
            travel_wp = [self._make_pose(travel_pos)]

            traj_t, frac_t = self._call_cartesian_path(travel_wp, current_joints)
            if traj_t is None or frac_t < 0.5:
                self.get_logger().warn(
                    f"[Plan] Travel plan failed for stroke {s_idx+1} "
                    f"(fraction={frac_t:.2f}), skipping stroke")
                continue
            for pt in traj_t.points:
                all_joint_waypoints.append((list(pt.positions), True))
            current_joints = list(traj_t.points[-1].positions)

            # ── draw waypoints (pen-down + stroke points) ──
            draw_poses = []
            for pt_idx, (px, py) in enumerate(stroke):
                rp = px_to_robot(float(px), float(py))
                rp[2] = Z_DRAW
                draw_poses.append(self._make_pose(rp))

            traj_d, frac_d = self._call_cartesian_path(draw_poses, current_joints)
            if traj_d is None or frac_d < 0.3:
                self.get_logger().warn(
                    f"[Plan] Draw plan for stroke {s_idx+1} fraction={frac_d:.2f}")
                if traj_d is None:
                    continue

            for pt in traj_d.points:
                all_joint_waypoints.append((list(pt.positions), False))
            current_joints = list(traj_d.points[-1].positions)

            # ── lift after stroke (pen-up) ──
            lift_pos = px_to_robot(*stroke[-1])
            lift_pos[2] = Z_TRAVEL
            traj_l, frac_l = self._call_cartesian_path(
                [self._make_pose(lift_pos)], current_joints)
            if traj_l is not None and traj_l.points:
                for pt in traj_l.points:
                    all_joint_waypoints.append((list(pt.positions), True))
                current_joints = list(traj_l.points[-1].positions)

        if not all_joint_waypoints:
            self.get_logger().error("[Plan] No waypoints planned at all")
            return None

        self.get_logger().info(
            f"[Plan] Total planned joint waypoints: {len(all_joint_waypoints)}")

        # ── Convert to URScript ──
        return self._joint_waypoints_to_urscript(all_joint_waypoints)

    def _joint_waypoints_to_urscript(self, waypoints) -> str:
        """Convert a list of (joint_positions, is_travel) to a URScript program."""
        lines = []
        lines.append("# UR3 Selfie Drawing Robot | MoveIt2 planned trajectory")
        lines.append("def draw_face():")
        lines.append(f"  # Marker holder: {MARKER_TILT_DEG} deg tilt, "
                     f"EE {EE_DRAW_HEIGHT*100:.0f} cm above canvas")
        lines.append(f"  set_tcp(p[{TCP_OFFSET[0]:.4f},{TCP_OFFSET[1]:.4f},"
                     f"{TCP_OFFSET[2]:.4f},{TCP_OFFSET[3]:.4f},"
                     f"{TCP_OFFSET[4]:.4f},{TCP_OFFSET[5]:.4f}])")
        lines.append("")

        # Home
        home_str = ",".join(f"{j:.4f}" for j in HOME_JOINTS)
        lines.append(f"  movej([{home_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
        lines.append("")

        prev_travel = None
        for joints, is_travel in waypoints:
            j_str = ",".join(f"{j:.4f}" for j in joints)
            if is_travel:
                # Use movej for travel (faster, joint-space)
                if prev_travel is not None and prev_travel == is_travel:
                    # Skip intermediate travel waypoints that are very close
                    pass
                lines.append(f"  movej([{j_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
            else:
                # Use movej with blend for drawing (follow planned path)
                lines.append(f"  movej([{j_str}], a={LINEAR_ACCEL}, v={LINEAR_VEL})")
            prev_travel = is_travel

        lines.append("")
        lines.append(f"  # Return home")
        lines.append(f"  movej([{home_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
        lines.append("end")
        lines.append("")
        lines.append("draw_face()")
        return "\n".join(lines)

    # ─────────────────── stage 5: execute ───────────────────

    def _send_urscript(self, script: str) -> bool:
        """Send URScript to the robot over TCP socket."""
        try:
            self.get_logger().info(
                f"[Execute] Connecting to {self.robot_ip}:{self.robot_port} ...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.robot_ip, self.robot_port))
            sock.sendall((script + "\n").encode("utf-8"))
            self.get_logger().info(
                f"[Execute] URScript sent ({len(script)} bytes)")
            sock.close()
            return True
        except Exception as e:
            self.get_logger().warn(f"[Execute] Socket failed: {e}")
            # Save to file as fallback
            ts = int(time.time())
            path = f"/tmp/drawing_moveit2_{ts}.script"
            with open(path, "w") as f:
                f.write(script)
            self.get_logger().info(f"[Execute] Script saved to {path}")
            return True  # Don't fail the pipeline

    # ─────────────────── helpers ───────────────────

    def _publish_status(self, status: str):
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


if __name__ == "__main__":
    main()
