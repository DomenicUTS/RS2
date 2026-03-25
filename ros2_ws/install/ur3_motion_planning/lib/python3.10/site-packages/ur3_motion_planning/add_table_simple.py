#!/usr/bin/env python3
"""
Simple script to add table collision object to MoveIt2 planning scene.
No heavy dependencies - just publishes to /collision_object topic.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import time


class TablePublisher(Node):
    def __init__(self):
        super().__init__('table_publisher')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        
    def add_table(self):
        """Publish table collision object."""
        # Create collision object
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = "world"
        collision_obj.id = "table"
        collision_obj.operation = CollisionObject.ADD
        
        # Set pose
        collision_obj.pose.position.x = 0.0
        collision_obj.pose.position.y = 0.0
        collision_obj.pose.position.z = -0.07
        collision_obj.pose.orientation.w = 1.0
        
        # Create box primitive
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [1.7, 1.7, 0.2]  # x, y, z
        
        collision_obj.primitives.append(primitive)
        
        # Publish
        self.publisher.publish(collision_obj)
        self.get_logger().info("✓ Table collision object published!")


def main(args=None):
    rclpy.init(args=args)
    
    # Create node and add table
    node = TablePublisher()
    time.sleep(0.5)  # Wait for connection
    
    # Publish table
    node.add_table()
    
    # Keep node alive briefly
    rclpy.spin_once(node, timeout_sec=1)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
