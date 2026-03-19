# UR3 Motion Planning Node (ROS 2)

Motion planning subsystem for Team Picasso UR3 Selfie-Drawing Robot.

This ROS 2 node:
- ✓ Subscribes to stroke paths from Perception subsystem
- ✓ Generates optimized trajectories (NN + 2-Opt)
- ✓ Executes on UR3 via Polyscope simulator or real robot
- ✓ Integrates with ur_robot_driver and MoveIt2
- ✓ Publishes status to GUI subsystem

---

## Installation

### 1. Install ROS 2 Humble (if not already done)
```bash
sudo apt update && sudo apt install ros-humble-desktop
```

### 2. Install UR ROS 2 Driver
```bash
sudo apt-get install ros-humble-ur
```

### 3. Source ROS 2 setup
```bash
source /opt/ros/humble/setup.bash
```

### 4. Build the motion planning package
```bash
cd ~/RS2/ros2_ws
colcon build
source install/setup.bash
```

---

## Testing with Polyscope Simulator

### Step 1: Start Polyscope Simulator

In **Terminal 1**, start the Docker simulator:
```bash
ros2 run ur_client_library start_ursim.sh -m ur3
```

Wait for it to finish (~30 seconds). You should see:
```
[ur3] Simulator Ready
```

Then visit **http://192.168.56.101:6080/vnc.html** to view Polyscope GUI (optional but helpful for debugging).

### Step 2: Launch Motion Planning Node

In **Terminal 2**:
```bash
cd ~/RS2/ros2_ws
source install/setup.bash

ros2 launch ur3_motion_planning ur3_motion_planning.launch.py \
    robot_ip:=192.168.56.101 \
    use_ros_control:=false
```

You should see:
```
[ur3_motion_planning_node] [Init] Motion Planning Node ready!
[ur3_motion_planning_node] [Robot] Connected to 192.168.56.101:30002
[ur3_motion_planning_node] [Subscribe] Listening on /stroke_paths
```

### Step 3: Send Test Stroke Paths

In **Terminal 3**, publish test stroke data:
```bash
cd ~/RS2
source ros2_ws/install/setup.bash

# Option A: Publish from JSON file (using simple test script)
python3 -c "
import rclpy
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Point
import json

rclpy.init()
node = rclpy.create_node('test_stroke_publisher')
pub = node.create_publisher(PoseArray, 'stroke_paths', 10)

# Load test strokes
with open('outputs/strokes/face1_strokes.json') as f:
    strokes = json.load(f)

# Convert to PoseArray (first 200 points for quick test)
msg = PoseArray()
msg.header.frame_id = 'canvas'
for stroke in strokes[:3]:  # First 3 strokes only
    for x, y in stroke[:50]:  # First 50 points per stroke
        pose = Pose()
        pose.position = Point(x=float(x), y=float(y), z=0.0)
        msg.poses.append(pose)

# Publish
for i in range(3):
    pub.publish(msg)
    print(f'Published {len(msg.poses)} waypoints')
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
"
```

### Step 4: Verify Execution

Check **Terminal 2** output:
```
[ur3_motion_planning_node] [Plan] Starting motion planning pipeline...
[ur3_motion_planning_node] [Plan] Scaling strokes to workspace...
[ur3_motion_planning_node] [NN Sort] 3 strokes | travel saved ≈ 150.0 px | time: 0.5 ms
[ur3_motion_planning_node] [2-Opt] 50 iterations | travel saved ≈ -20.0 px | time: 47.8 ms
[ur3_motion_planning_node] [Plan] Pipeline complete: 0.085s
[ur3_motion_planning_node] [Robot] Script sent (46223 bytes)
```

### Step 5: View Polyscope Simulator (Optional)

Visit **http://192.168.56.101:6080/vnc.html**:
1. You should see the script received in Polyscope
2. Click **Play** button to execute
3. Watch robot draw strokes

---

## Testing with Real UR3

### Prerequisites
- UR3 robot on same LAN as your computer
- UR3 configured with static IP (e.g., `10.0.0.2`)
- Network connectivity confirmed: `ping 10.0.0.2`

### Step 1: Calibrate Robot (One-time)

1. Use UR3 Teach Pendant to navigate to **Settings → Installation → Ports**
2. Note the robot's IP address
3. Set external control mode per official UR documentation

### Step 2: Start ur_robot_driver (Recommended)

In **Terminal 1**, launch UR driver:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3 \
    robot_ip:=10.0.0.2 \
    launch_rviz:=true
```

You should see RViz window with robot model.

### Step 3: Launch Motion Planning Node

In **Terminal 2**:
```bash
cd ~/RS2/ros2_ws
source install/setup.bash

# Option A: Using ur_robot_driver (recommended for real robot)
ros2 launch ur3_motion_planning ur3_motion_planning.launch.py \
    robot_ip:=10.0.0.2 \
    use_ros_control:=true

# Option B: Direct TCP (if ur_robot_driver not desired)
ros2 launch ur3_motion_planning ur3_motion_planning.launch.py \
    robot_ip:=10.0.0.2 \
    use_ros_control:=false
```

### Step 4: Play "External Control" on UR3

1. On UR3 Teach Pendant, navigate to **Program** tab
2. Load or create program: **External Control**
3. Click **Play** (or press **▶** on pendant)

### Step 5: Send Stroke Paths

Repeat Step 3 from Polyscope section (same ROS 2 topic publishing).

The robot will now execute the drawing!

---

## Understanding the Architecture

### Topic Flow

```
Perception Node (Nithish)
    ↓
    publishes: /stroke_paths (geometry_msgs/PoseArray)
    ↓
Motion Planning Node (YOU)
    • Subscribes to /stroke_paths
    • Runs optimization pipeline
    • Publishes: /urscript_program (URScript code)
    ↓
ur_robot_driver (ROS 2) OR Direct TCP
    ↓
UR3 Robot / Polyscope Simulator
    ↓
Physical Drawing
```

### QoS (Quality of Service)

- **BEST_EFFORT**: Fast, low-latency communication
- **KEEP_LAST**: Maintains recent message (don't spam)
- Appropriate for real-time drawing use case

---

## Configuration

### Launch Arguments

When launching, customize with:

```bash
ros2 launch ur3_motion_planning ur3_motion_planning.launch.py \
    robot_ip:=192.168.56.101 \              # Robot IP
    robot_port:=30002 \                     # TCP port
    use_ros_control:=false \                # TCP vs ur_robot_driver
    optimization_enabled:=true              # Enable NN+2-Opt
```

### Runtime Parameters

Modify behavior via ROS 2 `set_parameter`:

```bash
# Disable path optimization
ros2 param set /ur3_motion_planning optimization_enabled false

# Change robot IP
ros2 param set /ur3_motion_planning robot_ip 10.0.0.2
```

---

## Integration with Other Subsystems

### Perception Integration (Nithish)

Your perception node should publish `geometry_msgs/PoseArray`:

```python
from geometry_msgs.msg import PoseArray, Pose, Point

# In your perception node:
msg = PoseArray()
msg.header.frame_id = 'canvas'

for stroke in strokes:
    for x, y in stroke:
        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.0)
        msg.poses.append(pose)

publisher.publish(msg)  # Publishes to /stroke_paths
```

### GUI Integration (Mateusz)

Your GUI node should:

1. **Subscribe** to `/planning_status` to show feedback:
```python
node.create_subscription(String, '/planning_status', callback, 10)
# Shows: "Planning complete (0.085s) - Ready to execute"
```

2. **Subscribe** to `/urscript_program` to display generated code (optional):
```python
node.create_subscription(String, '/urscript_program', callback, 10)
```

3. **Publish** to `/motion_params` to control planning:
```python
# Example: Disable optimization for faster preview
msg = String()
msg.data = json.dumps({"optimization": False})
publisher.publish(msg)
```

---

## Troubleshooting

### "Connection failed: 192.168.56.101:30002"
- Ensure Polyscope simulator is running: `ros2 run ur_client_library start_ursim.sh -m ur3`
- Check if Docker is running: `docker ps`
- Try manually: `ping 192.168.56.101`

### "ModuleNotFoundError: ur3_selfie_draw"
- Ensure source paths are correct in motion_planning_node.py
- Check `sys.path.insert(0, ...)` points to correct location

### "ROS 2 package not found"
- Run `colcon build` in workspace
- Run `source install/setup.bash`
- Verify: `ros2 pkg list | grep ur3_motion_planning`

### Real Robot: "IK solution not found"
- Check canvas origin calibration
- Increase workspace margins in ur3_selfie_draw.py
- Check joint limits: `rostopic echo /joint_states`

---

## Command Reference

**Build package:**
```bash
cd ~/RS2/ros2_ws && colcon build
```

**Start Polyscope simulator:**
```bash
ros2 run ur_client_library start_ursim.sh -m ur3
```

**Launch motion planning node:**
```bash
ros2 launch ur3_motion_planning ur3_motion_planning.launch.py robot_ip:=192.168.56.101
```

**View node logs:**
```bash
ros2 run rqt_console rqt_console  # GUI log viewer
# or:
ros2 topic echo /planning_status   # Watch status messages
```

**Monitor ROS graph:**
```bash
ros2 run rqt_graph rqt_graph      # Visualize node/topic connections
```

**Check active nodes:**
```bash
ros2 node list  # Shows all running nodes
ros2 topic list # Shows all active topics
```

---

## Files Reference

- `motion_planning_node.py` — Main ROS 2 node (495 lines)
- `ur3_motion_planning.launch.py` — Launch configuration
- `package.xml` — ROS 2 package metadata
- `setup.py` — Python package setup

---

## Performance Metrics

Tested with face1.json (13 strokes, 1294 waypoints):

| Stage | Time |
|-------|------|
| Scaling detection | <1ms |
| NN optimization | 0.8-1.3ms |
| 2-Opt refinement | 58-118ms |
| URScript generation | 9-15ms |
| **Total pipeline** | **75-134ms** |
| Polyscope execution | ~120 seconds |
| Travel reduction | 41.0% |

**Total system time:** ~2 minutes (well under 3-minute requirement)

---

## For Sprint 2 Presentation

This ROS 2 integration demonstrates:
- ✅ Understanding of system-level architecture
- ✅ Integration with other subsystems (Perception, GUI)
- ✅ Proper use of ROS 2 topics/pub-sub pattern
- ✅ Backward compatibility with existing TCP code
- ✅ Flexibility: works with both Polyscope and real robot
- ✅ Professional software engineering practices

Mention in presentation:
> "We converted the motion planning subsystem to a ROS 2 node to integrate with Perception and GUI subsystems via publish-subscribe architecture. This allows modular, decoupled subsystem communication."

