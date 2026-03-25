# MoveIt2 Motion Planning - Testing Guide

## Overview
The motion planning system now uses MoveIt2 for collision avoidance. The table is defined as a collision object to prevent the robot from hitting it.

---

## Testing in RViz Simulator (Recommended First)

### Terminal 1 - Start MoveIt2 + RViz

```bash
cd ~/RS2/ros2_ws
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit_config.launch.py ur_type:=ur3 launch_rviz:=true use_sim_time:=true
```

**Expected:**
- RViz window opens showing UR3 robot
- Motion Planning panel visible on left
- Table collision object defined in planning scene

### Terminal 2 - Launch Motion Planning Node

```bash
cd ~/RS2/ros2_ws
source install/setup.bash
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py use_sim_time:=true
```

**Expected output:**
```
[ur3_motion_planning_node] [Init] Motion Planning Node (MoveIt2) ready!
[MoveIt2] Move group initialized
[Planning Scene] Table collision object added
[Subscribe] Listening on /stroke_paths
```

### Terminal 3 - Send Test Strokes

```bash
cd ~/RS2
source ros2_ws/install/setup.bash
python3 << 'EOF'
import json, rclpy
from geometry_msgs.msg import PoseArray, Pose, Point

rclpy.init()
node = rclpy.create_node('test_pub')
pub = node.create_publisher(PoseArray, 'stroke_paths', 10)

with open('outputs/strokes/face1_strokes.json') as f:
    strokes = json.load(f)

msg = PoseArray()
msg.header.frame_id = 'canvas'
for stroke in strokes[:3]:
    for x, y in stroke:
        pose = Pose()
        pose.position = Point(x=float(x), y=float(y), z=0.0)
        msg.poses.append(pose)

pub.publish(msg)
print("✓ Sent test strokes to RViz simulator")
rclpy.shutdown()
EOF
```

**Expected in RViz:**
- Motion trajectory visualized in RViz
- Robot avoids table collision
- Motions stay above table surface

---

## Testing on Real Robot

### Terminal 1 - Start UR Driver + MoveIt2

```bash
cd ~/RS2/ros2_ws
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit_config.launch.py ur_type:=ur3 robot_ip:=192.168.0.196 launch_rviz:=true
```

### Terminal 2 - Launch Motion Planning Node

```bash
cd ~/RS2/ros2_ws
source install/setup.bash
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py robot_ip:=192.168.0.196
```

### Terminal 3 - Send Drawing Strokes

```bash
cd ~/RS2
source ros2_ws/install/setup.bash
python3 << 'EOF'
import json, rclpy
from geometry_msgs.msg import PoseArray, Pose, Point

rclpy.init()
node = rclpy.create_node('face1_drawer')
pub = node.create_publisher(PoseArray, 'stroke_paths', 10)

with open('outputs/strokes/face1_strokes.json') as f:
    strokes = json.load(f)

msg = PoseArray()
msg.header.frame_id = 'canvas'
for stroke in strokes:
    for x, y in stroke:
        pose = Pose()
        pose.position = Point(x=float(x), y=float(y), z=0.0)
        msg.poses.append(pose)

pub.publish(msg)
print(f"✓ Sent {len(msg.poses)} waypoints to real UR3")
rclpy.shutdown()
EOF
```

**On UR3 teach pendant:**
- Accept the command
- Press Play to execute
- Robot will plan collision-free path
- Drawing executes with table obstacle avoidance

---

## Troubleshooting

**"Move group failed to initialize"**
- Ensure ur_moveit_config is running first
- Check ROS 2 environment is sourced

**"Planning failed - path blocked"**
- Table collision bounds too large
- Waypoints unreachable from current position
- Try starting with fewer strokes

**"Robot jerks during execution"**
- Normal behavior - MoveIt2 replans at intervals
- Reduce LINEAR_VEL in motion_planning_node_moveit2.py if too aggressive

---

## Configuration

Edit in `motion_planning_node_moveit2.py`:
- `CANVAS_ORIGIN`: Canvas position in robot frame
- `Z_DRAW`: Drawing height (currently 0.01m at table surface)
- `Z_TRAVEL`: Pen-up height (currently 0.20m = 20cm above table)
- `LINEAR_VEL`: Drawing speed

Edit table collision in `_add_table_collision()`:
- `table_pose`: Position of table center
- `table_size`: [length, width, height] in meters

---

## ROS 2 Topics

**Subscribers:**
- `/stroke_paths` (geometry_msgs/PoseArray) - Waypoint coordinates

**Publishers:**
- `/planning_status` (std_msgs/String) - Planning state updates
- `/urscript_program` (std_msgs/String) - Generated commands (for logging)
