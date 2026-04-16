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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
import threading
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene
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
HOME_JOINTS = [1.047, -1.57, 1.57, -1.57, -1.57, 0.0]  # shoulder_pan rotated 60° to face canvas


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
        self._pipeline_thread = None
        self._current_joints = list(HOME_JOINTS)  # track planned joint state

        # ── Callback groups ──
        self._service_cb_group = ReentrantCallbackGroup()

        # ── Publishers / Subscribers ──
        self.status_pub = self.create_publisher(String, "drawing_status", 10)
        self.trajectory_display_pub = self.create_publisher(PoseArray, "trajectory_preview", 10)
        self.strokes_sub = self.create_subscription(
            String, "drawing_strokes", self._on_drawing_strokes, 10)
        self.gui_cmd_sub = self.create_subscription(
            String, "gui/command", self._on_gui_command, 10)

        # ── Joint state publisher (replaces joint_state_publisher node) ──
        # Publishes the current planned joint positions so MoveIt2 and RViz
        # show a single, consistent robot model (eliminates phantom robot).
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._js_timer = self.create_timer(0.1, self._publish_joint_states)  # 10 Hz

        # ── MoveIt2 service client (on separate callback group) ──
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path",
            callback_group=self._service_cb_group)

        # ── Scene service client ──
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene',
            callback_group=self._service_cb_group)

        self.get_logger().info("[Init] UR3 Drawing Node (MoveIt2 planning mode)")
        self.get_logger().info(f"[Config] Robot: {self.robot_ip}:{self.robot_port}")
        self.get_logger().info(f"[Config] Face: {self.face} | Source: {self.stroke_source}")
        self.get_logger().info(f"[Config] MoveIt2 max_step={self.max_step} m, "
                               f"jump_threshold={self.jump_threshold}")

        if self.stroke_source == "file":
            self.create_timer(2.0, self._file_startup_timer)
        else:
            self.get_logger().info("[Init] Waiting for strokes on /drawing_strokes ...")
            self._publish_status("WAITING_FOR_PERCEPTION")

    # ─────────────────── callbacks ───────────────────

    def _publish_joint_states(self):
        """Publish current planned joint positions on /joint_states at 10 Hz.
        This keeps MoveIt2 and RViz synchronized with our planned state,
        preventing the phantom robot issue."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(j) for j in self._current_joints]
        self.joint_state_pub.publish(msg)

    def _file_startup_timer(self):
        """One-shot timer callback for file-based stroke source."""
        self._start_pipeline_thread()

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
            self._start_pipeline_thread()

    def _on_gui_command(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"[GUI] Command: {cmd}")
        if cmd == "START" and self._topic_strokes is not None:
            self._startup_done = False
            self._start_pipeline_thread()
        elif cmd == "STOP":
            self._publish_status("STOPPED")
        elif cmd == "PAUSE":
            self._publish_status("PAUSED")
        elif cmd == "RESUME":
            self._publish_status("RESUMED")

    def _start_pipeline_thread(self):
        """Run the pipeline on a background thread so service calls don't deadlock."""
        if self._pipeline_thread is not None and self._pipeline_thread.is_alive():
            self.get_logger().warn("[Pipeline] Already running, ignoring")
            return
        self._pipeline_thread = threading.Thread(
            target=self._on_startup_complete, daemon=True)
        self._pipeline_thread.start()

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

            # 3 – Scene objects already applied by add_table node (launched earlier)
            #     Skip redundant publish to avoid spin deadlock.
            self._publish_status("SETTING_UP_SCENE")
            self.get_logger().info("[Scene] Using collision objects from add_table node")

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

    def _apply_scene(self, scene_msg: PlanningScene) -> bool:
        """Apply a PlanningScene diff via the /apply_planning_scene service."""
        if not self.apply_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('[Scene] /apply_planning_scene service not available')
            return False
        req = ApplyPlanningScene.Request()
        req.scene = scene_msg
        future = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().success:
            return True
        self.get_logger().error('[Scene] ApplyPlanningScene call failed')
        return False

    def _publish_scene_objects(self):
        """Apply table and marker-holder collision objects via service."""

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

        scene1 = PlanningScene()
        scene1.is_diff = True
        scene1.world.collision_objects.append(table)
        if self._apply_scene(scene1):
            self.get_logger().info("[Scene] Table applied")

        # Marker holder attached to tool0 — top face flush with flange,
        # full height extends downward along the tilted marker axis.
        holder_co = CollisionObject()
        holder_co.header.frame_id = "tool0"
        holder_co.id = "marker_holder"
        holder_co.operation = CollisionObject.ADD
        hbox = SolidPrimitive()
        hbox.type = SolidPrimitive.BOX
        hbox.dimensions = [0.160, 0.180, EE_DRAW_HEIGHT]
        holder_co.primitives.append(hbox)
        full = EE_DRAW_HEIGHT
        hp = Pose()
        hp.position.x = full * math.sin(MARKER_TILT_RAD)
        hp.position.z = -full * math.cos(MARKER_TILT_RAD)
        hp.orientation.y = math.sin(MARKER_TILT_RAD / 2.0)
        hp.orientation.w = math.cos(MARKER_TILT_RAD / 2.0)
        holder_co.primitive_poses.append(hp)

        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = holder_co
        attached.touch_links = ["tool0", "wrist_3_link"]

        scene2 = PlanningScene()
        scene2.is_diff = True
        scene2.robot_state.attached_collision_objects.append(attached)
        scene2.robot_state.is_diff = True
        if self._apply_scene(scene2):
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

        # Wait for the future (executed from a background thread,
        # the MultiThreadedExecutor handles the response callback)
        deadline = time.time() + self.planning_timeout
        while not future.done():
            if time.time() > deadline:
                self.get_logger().error("[Plan] Cartesian path service call timed out")
                return None, 0.0
            time.sleep(0.01)

        if not future.done() or future.result() is None:
            self.get_logger().error("[Plan] Cartesian path service call failed")
            return None, 0.0

        resp = future.result()
        fraction = resp.fraction
        traj = resp.solution.joint_trajectory

        return traj, fraction

    def _thin_trajectory(self, traj_points, is_draw=False, max_joint_delta=0.05):
        """
        Thin a MoveIt2 joint trajectory while preserving collision safety.

        For travel/lift: keep only the final point (single movej at Z_TRAVEL).
        For drawing: keep ALL MoveIt2-planned points to preserve the exact
                     collision-free Cartesian path.  Previously, aggressive
                     thinning caused joint-space shortcuts that made the
                     end-effector deviate (up/down oscillations).
        """
        n = len(traj_points)
        if n == 0:
            return []
        if n <= 2:
            return [list(pt.positions) for pt in traj_points]

        if not is_draw:
            # Travel/lift: single movej to endpoint (robot is high up, safe)
            return [list(traj_points[-1].positions)]

        # Drawing: keep ALL MoveIt2 points so the robot follows the exact
        # collision-free Cartesian path without joint-space shortcuts.
        return [list(pt.positions) for pt in traj_points]

    def _plan_and_build_urscript(self, strokes) -> str:
        """
        For each stroke, plan a collision-free trajectory via MoveIt2,
        then use the actual planned joint positions in URScript movej commands.
        This ensures the robot follows MoveIt2's collision-free path.
        """

        all_segments = []  # list of (joint_positions_list, is_travel, comment)
        current_joints = list(HOME_JOINTS)
        total_strokes = len(strokes)
        skipped = 0

        for s_idx, stroke in enumerate(strokes):
            if not stroke:
                continue
            self.get_logger().info(
                f"[Plan] Stroke {s_idx+1}/{total_strokes} "
                f"({len(stroke)} pts) ...")

            # ── Travel to above first point (pen-up) ──
            travel_pos = px_to_robot(*stroke[0])
            travel_pos[2] = Z_TRAVEL
            travel_wp = [self._make_pose(travel_pos)]

            traj_t, frac_t = self._call_cartesian_path(travel_wp, current_joints)
            if traj_t is None or frac_t < 0.5:
                self.get_logger().warn(
                    f"[Plan] Travel plan failed for stroke {s_idx+1} "
                    f"(fraction={frac_t:.2f}), skipping stroke")
                skipped += 1
                continue
            current_joints = list(traj_t.points[-1].positions)
            self._current_joints = list(current_joints)  # update for joint state publisher

            # Use MoveIt2's planned joints for travel (just final point)
            travel_joints = self._thin_trajectory(traj_t.points, is_draw=False)
            for jts in travel_joints:
                all_segments.append((jts, True, f"travel s{s_idx+1}"))

            # ── Draw waypoints (pen-down + stroke points) ──
            draw_poses = []
            for px, py in stroke:
                rp = px_to_robot(float(px), float(py))
                rp[2] = Z_DRAW
                draw_poses.append(self._make_pose(rp))

            traj_d, frac_d = self._call_cartesian_path(draw_poses, current_joints)
            if traj_d is None or frac_d < 0.3:
                self.get_logger().warn(
                    f"[Plan] Draw plan for stroke {s_idx+1} fraction={frac_d:.2f}")
                if traj_d is None:
                    skipped += 1
                    continue

            current_joints = list(traj_d.points[-1].positions)
            self._current_joints = list(current_joints)  # update for joint state publisher
            draw_joints = self._thin_trajectory(traj_d.points, is_draw=True)
            for i, jts in enumerate(draw_joints):
                label = "pen-down" if i == 0 else f"draw s{s_idx+1}"
                all_segments.append((jts, False, label))

            # ── Lift after stroke (pen-up) ──
            lift_pos = px_to_robot(*stroke[-1])
            lift_pos[2] = Z_TRAVEL
            traj_l, frac_l = self._call_cartesian_path(
                [self._make_pose(lift_pos)], current_joints)
            if traj_l is not None and traj_l.points:
                current_joints = list(traj_l.points[-1].positions)
                self._current_joints = list(current_joints)  # update for joint state publisher
                lift_joints = self._thin_trajectory(traj_l.points, is_draw=False)
                for jts in lift_joints:
                    all_segments.append((jts, True, f"pen-up s{s_idx+1}"))

        if not all_segments:
            self.get_logger().error("[Plan] No waypoints planned at all")
            return None

        self.get_logger().info(
            f"[Plan] {len(all_segments)} movej commands "
            f"({skipped} strokes skipped)")

        # ── Build URScript ──
        return self._build_urscript(all_segments)

    def _build_urscript(self, segments) -> str:
        """Build a complete URScript program from MoveIt2-planned joint waypoints."""
        lines = []
        # NOTE: Script must start with 'def' — no leading comments.
        # Some CB3 firmware silently rejects scripts with text before 'def'.
        lines.append("def draw_face():")
        lines.append(f"  set_tcp(p[{TCP_OFFSET[0]:.4f},{TCP_OFFSET[1]:.4f},"
                     f"{TCP_OFFSET[2]:.4f},{TCP_OFFSET[3]:.4f},"
                     f"{TCP_OFFSET[4]:.4f},{TCP_OFFSET[5]:.4f}])")
        lines.append("")

        # Home via joint move (safe from any position)
        home_str = ",".join(f"{j:.4f}" for j in HOME_JOINTS)
        lines.append(f"  movej([{home_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
        lines.append("")

        for idx, (joints, is_travel, comment) in enumerate(segments):
            j_str = ",".join(f"{j:.4f}" for j in joints)
            if is_travel:
                # Travel/lift: faster joint-space move, no blend
                lines.append(f"  movej([{j_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
            else:
                # Drawing: use blend radius for smooth continuous motion
                # except for the very last drawing point (blend=0 to stop precisely)
                is_last_draw = (idx + 1 >= len(segments) or segments[idx + 1][1])
                if is_last_draw:
                    lines.append(f"  movej([{j_str}], a=0.5, v=0.3)")
                else:
                    lines.append(f"  movej([{j_str}], a=0.5, v=0.3, r=0.002)")

        lines.append("")
        lines.append(f"  movej([{home_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
        lines.append("end")
        return "\n".join(lines)

    # ─────────────────── stage 5: execute ───────────────────

    def _send_urscript(self, script: str) -> bool:
        """Send URScript to the robot over TCP socket."""

        # Always save the script for inspection / manual replay
        save_path = os.path.expanduser("~/RS2/outputs/last_drawing.script")
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "w") as f:
            f.write(script)
        line_count = script.count('\n') + 1
        self.get_logger().info(
            f"[Execute] Script saved to {save_path} "
            f"({len(script)} bytes, {line_count} lines)")

        try:
            self.get_logger().info(
                f"[Execute] Connecting to {self.robot_ip}:{self.robot_port} ...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            sock.connect((self.robot_ip, self.robot_port))
            self.get_logger().info("[Execute] Connected to robot")

            encoded = (script + "\n").encode("utf-8")
            sock.sendall(encoded)
            self.get_logger().info(
                f"[Execute] URScript sent ({len(encoded)} bytes). "
                f"Waiting for robot to start executing ...")

            # Keep socket open and monitor robot state for up to 10 seconds
            # to give the CB3 controller time to parse and start execution.
            # Closing too early can cause the controller to abort on real hardware.
            sock.settimeout(2.0)
            started_waiting = time.time()
            while time.time() - started_waiting < 10.0:
                try:
                    resp = sock.recv(4096)
                    if resp:
                        self.get_logger().info(
                            f"[Execute] Robot state packet ({len(resp)} bytes)")
                except socket.timeout:
                    pass
                # Check if enough time has passed for the controller to start
                if time.time() - started_waiting >= 5.0:
                    break

            sock.close()
            self.get_logger().info(
                "[Execute] Done — socket closed after monitoring period.\n"
                "  If robot is not moving, check:\n"
                f"  1. Robot is powered ON and initialized\n"
                f"  2. Robot is in 'Remote Control' mode (not 'Local')\n"
                f"  3. Robot is not in Protective Stop / Emergency Stop\n"
                f"  4. Review script: cat {save_path}")
            return True
        except ConnectionRefusedError:
            self.get_logger().error(
                f"[Execute] CONNECTION REFUSED at {self.robot_ip}:{self.robot_port}. "
                f"Is the robot running? Is the robot IP correct?")
            return False
        except socket.timeout:
            self.get_logger().error(
                f"[Execute] CONNECTION TIMED OUT to {self.robot_ip}:{self.robot_port}. "
                f"Check network connectivity to the robot/simulator.")
            return False
        except Exception as e:
            self.get_logger().error(f"[Execute] Socket error: {type(e).__name__}: {e}")
            return False

    # ─────────────────── helpers ───────────────────

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR3DrawingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
