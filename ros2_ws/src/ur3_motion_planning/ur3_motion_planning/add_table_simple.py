#!/usr/bin/env python3
"""
Add table + marker-holder collision objects to the MoveIt2 planning scene.

Objects:
  1. table       – free-standing box below the robot base
  2. marker_holder – 160 × 180 mm rectangular prism ATTACHED to tool0
                     (moves with the end effector so MoveIt plans around it)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
import math
import time

# ── Marker holder geometry (must match ur3_selfie_draw.py) ──
MARKER_TILT_DEG = 20.0
MARKER_TILT_RAD = math.radians(MARKER_TILT_DEG)
EE_DRAW_HEIGHT  = 0.15  # m – end-effector height above canvas


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)
        self.attached_pub = self.create_publisher(
            AttachedCollisionObject, '/attached_collision_object', 10)

    # ── Table ──────────────────────────────────────────────────
    def add_table(self):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = "table"
        obj.operation = CollisionObject.ADD

        obj.pose.position.x = 0.0
        obj.pose.position.y = 0.0
        obj.pose.position.z = -0.07
        obj.pose.orientation.w = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.7, 1.7, 0.2]
        obj.primitives.append(box)

        self.collision_pub.publish(obj)
        self.get_logger().info("Table collision object published")

    # ── Marker holder (attached to tool0) ─────────────────────
    def add_marker_holder(self):
        """
        Attach a 160 × 180 mm rectangular prism to the end effector.
        The holder extends downward along the tilted marker axis from
        the tool0 flange frame.
        """
        # Inner collision object describing the shape
        obj = CollisionObject()
        obj.header.frame_id = "tool0"
        obj.id = "marker_holder"
        obj.operation = CollisionObject.ADD

        # Prism dimensions: 0.160 m × 0.180 m × EE_DRAW_HEIGHT
        # (height = distance from flange to marker tip along holder axis)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.160, 0.180, EE_DRAW_HEIGHT]
        obj.primitives.append(box)

        # Pose in tool0 frame – centre of the box is halfway along the
        # tilted holder axis (Y-axis tilt in the tool0 local frame).
        half = EE_DRAW_HEIGHT / 2.0
        pose = Pose()
        pose.position.x = half * math.sin(MARKER_TILT_RAD)   # forward
        pose.position.y = 0.0
        pose.position.z = -half * math.cos(MARKER_TILT_RAD)  # downward
        # Orientation: tilt around Y by MARKER_TILT_RAD
        pose.orientation.x = 0.0
        pose.orientation.y = math.sin(MARKER_TILT_RAD / 2.0)
        pose.orientation.z = 0.0
        pose.orientation.w = math.cos(MARKER_TILT_RAD / 2.0)
        obj.primitive_poses.append(pose)

        # Wrap in AttachedCollisionObject
        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = obj
        # Allow the holder to touch the wrist links it is bolted to
        attached.touch_links = ["tool0", "wrist_3_link"]

        self.attached_pub.publish(attached)
        self.get_logger().info(
            f"Marker holder attached to tool0 "
            f"(160x180x{EE_DRAW_HEIGHT*1000:.0f} mm, tilt={MARKER_TILT_DEG}°)")


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()

    # Wait for publishers to be discovered
    time.sleep(1.0)

    node.add_table()
    time.sleep(0.3)
    node.add_marker_holder()

    # Keep alive long enough for move_group to ingest the messages
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
