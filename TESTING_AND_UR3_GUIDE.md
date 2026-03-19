# Quick Testing Guide: ROS 2 Motion Planning Node

## Overview

Your motion planning code now works as a **ROS 2 node** that:
- Listens for stroke paths from your teammates' perception subsystem
- Generates optimized trajectories
- Publishes URScript to Polyscope or real UR3
- Sends status updates to your teammates' GUI

**Instead of:** `face1.json` → `ur3_selfie_draw.py` → TCP socket  
**Now:** `/stroke_paths` topic → motion planning node → `/urscript_program` + robot control

---

## Testing on Polyscope Simulator (10 minutes)

### Terminal 1: Start Simulator
```bash
ros2 run ur_client_library start_ursim.sh -m ur3
```
**Wait 30 seconds** until you see "Simulator Ready" (Docker container starting)

### Terminal 2: Build and Launch Node
```bash
cd ~/RS2/ros2_ws
colcon build
source install/setup.bash
ros2 launch ur3_motion_planning ur3_motion_planning.launch.py robot_ip:=192.168.56.101
```

**Expected output:**
```
[ur3_motion_planning_node] [Init] Motion Planning Node ready!
[ur3_motion_planning_node] [Robot] Connected to 192.168.56.101:30002
[ur3_motion_planning_node] [Subscribe] Listening on /stroke_paths
```

✅ **If you see these lines = Success!**

### Terminal 3: Send Test Data
```bash
cd ~/RS2
source ros2_ws/install/setup.bash

python3 <<'EOF'
import rclpy
from geometry_msgs.msg import PoseArray, Pose, Point
import json

rclpy.init()
node = rclpy.create_node('test_publisher')
pub = node.create_publisher(PoseArray, 'stroke_paths', 10)

# Load test strokes from your existing JSON
with open('outputs/strokes/face1_strokes.json') as f:
    strokes = json.load(f)

# Convert to ROS message (take first 3 strokes for quick test)
msg = PoseArray()
msg.header.frame_id = 'canvas'
for stroke in strokes[:3]:
    for x, y in stroke[:100]:  # First 100 points
        pose = Pose()
        pose.position = Point(x=float(x), y=float(y), z=0.0)
        msg.poses.append(pose)

# Publish
pub.publish(msg)
print(f"✓ Published {len(msg.poses)} waypoints to /stroke_paths")

rclpy.spin_once(node, timeout_sec=1)
node.destroy_node()
rclpy.shutdown()
EOF
```

### Terminal 2: Check Output
You should see (in Terminal 2):
```
[ur3_motion_planning_node] [Stroke] Received 150 waypoints
[ur3_motion_planning_node] [Plan] Starting motion planning pipeline...
[ur3_motion_planning_node] [NN Sort] 3 strokes | travel saved ≈ 150.0 px | time: 0.5 ms
[ur3_motion_planning_node] [2-Opt] 50 iterations | travel saved ≈ -20.0 px | time: 47.8 ms
[ur3_motion_planning_node] [Plan] Pipeline complete: 0.085s
[ur3_motion_planning_node] [Robot] Script sent (46223 bytes)
```

✅ **Script was sent to Polyscope successfully!**

### Optional: View Polyscope GUI
Open browser: **http://192.168.56.101:6080/vnc.html**
- Click VNC to view Polyscope desktop
- In Polyscope, look for "Call draw_face()" program (your generated script)
- Click **Play** to watch robot draw

---

## Verifying It Still Works Like Before

**Your code is unchanged!**

The motion planning functions (`scale_strokes_to_workspace`, `nearest_neighbour_sort`, `two_opt_improve`, `build_urscript`) are exactly the same.

The only difference:
- **Before:** Read from `face1.json` file
- **Now:** Receives strokes via ROS 2 topic `/stroke_paths`

**Same output:** URScript program sent to robot

---

## Connecting to Real UR3

### Prerequisites
1. UR3 robot on same network as your computer
2. UR3 IP address (e.g., `10.0.0.2` - check with IT/lab staff)
3. Network connectivity: `ping 10.0.0.2` should work
4. UR3 configured for "External Control" mode (lab staff will set this up)

### Step 1: Find UR3 IP Address

**Ask your tutor or check the network:**
- On UR3 Teach Pendant: **Settings** → **System** → check IP
- Common IPs: `10.0.0.2`, `192.168.1.X`, `172.16.0.X`

Let's say it's **`10.0.0.2`**

### Step 2: Test Network Connectivity
```bash
ping 10.0.0.2
# Should show: bytes from 10.0.0.2... (not "Unreachable")
```

### Step 3: Launch Motion Planning for Real Robot

**Terminal 1: Start UR Driver (Recommended)**
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3 \
    robot_ip:=10.0.0.2 \
    launch_rviz:=true
```

Should show RViz window with robot model.

**Terminal 2: Launch Motion Planning Node**
```bash
cd ~/RS2/ros2_ws
source install/setup.bash

ros2 launch ur3_motion_planning ur3_motion_planning.launch.py \
    robot_ip:=10.0.0.2 \
    use_ros_control:=true
```

### Step 4: Activate External Control on UR3

On the **UR3 Teach Pendant**:
1. Go to **Program** tab
2. Load program "External Control" (or create new one)
3. Press **Play** (▶ button)

Now your ROS 2 node can send commands!

### Step 5: Send Test Strokes (Same as Simulator)

Use Terminal 3 Python script (same as above) to publish stroke paths.

The real robot will execute them!

---

## Key Differences: Simulator vs Real Robot

| Aspect | Polyscope Simulator | Real UR3 |
|--------|---|---|
| **IP** | `192.168.56.101` (Docker) | `10.0.0.2` (actual robot) |
| **Connection** | TCP direct to port 30002 | TCP direct OR via ur_robot_driver |
| **Speed** | Fast (no physical constraints) | Limited (safe speed: ~0.1 m/s) |
| **Accuracy** | High (ideal physics) | Real-world variations |
| **Calibration** | Pre-set in Docker | Must calibrate canvas origin |

---

## Integration Points with Your Team

### With Perception (Nithish)
Nithish's node publishes strokes:
```python
# In Nithish's perception node:
pub = node.create_publisher(PoseArray, 'stroke_paths', 10)
pub.publish(pose_array_msg)  # Your node automatically receives this!
```

**No changes needed** - ROS 2 topic automatically connects them.

### With GUI (Mateusz)
Mateusz's node can:

1. **Send planning parameters:**
```python
# In Mateusz's GUI node:
params_pub = create_publisher(String, 'motion_params', 10)
params_pub.publish(String(data='{"optimization": true}'))
```

2. **Receive robot status:**
```python
# In Mateusz's GUI node:
def status_callback(msg):
    # msg.data = "Planning complete (0.085s) - Ready to execute"
    display_status_on_gui(msg.data)

status_sub = create_subscription(String, 'planning_status', status_callback, 10)
```

---

## Command Cheat Sheet

**Build:**
```bash
cd ~/RS2/ros2_ws && colcon build
```

**Polyscope Test:**
```bash
# Terminal 1:
ros2 run ur_client_library start_ursim.sh -m ur3

# Terminal 2:
cd ~/RS2/ros2_ws && source install/setup.bash
ros2 launch ur3_motion_planning ur3_motion_planning.launch.py robot_ip:=192.168.56.101
```

**Real Robot:**
```bash
# Terminal 1:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=10.0.0.2 launch_rviz:=true

# Terminal 2:
ros2 launch ur3_motion_planning ur3_motion_planning.launch.py robot_ip:=10.0.0.2 use_ros_control:=true
```

**Monitor Topics:**
```bash
ros2 topic echo /planning_status    # Watch status messages
ros2 topic echo /urscript_program   # See generated URScript (verbose!)
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "colcon: command not found" | `source /opt/ros/humble/setup.bash` |
| "ur3_motion_planning not found" | `colcon build` then `source install/setup.bash` |
| "Cannot connect to 192.168.56.101" | Simulator not running: `ros2 run ur_client_library start_ursim.sh -m ur3` |
| "Cannot connect to 10.0.0.2" | Check IP address: `ping 10.0.0.2` or ask lab staff |
| "Node spins but receives no strokes" | Perception node not running / not publishing to `/stroke_paths` |
| "IK solution not found on real robot" | Canvas origin calibration incorrect, or increased worksp margins needed |

---

## For Your Sprint 2 Presentation

Mention:
> "We converted the motion planning subsystem into a ROS 2 node that integrates with Perception and GUI via pub-sub topics. This allows parallel development and testing of subsystems independently."

Show:
1. Terminal output showing node connected
2. Topic graph showing `/stroke_paths` → motion planning → `/urscript_program`
3. Robot executing script (even if just simulator)

This demonstrates professional software architecture and teamwork readiness!

---

## Questions?

- Motion planning code: `~/RS2/src/ur3_selfie_draw.py` (unchanged)
- ROS 2 wrapper: `~/RS2/ros2_ws/src/ur3_motion_planning/ur3_motion_planning/motion_planning_node.py`
- Full docs: `~/RS2/ros2_ws/src/ur3_motion_planning/README.md`

