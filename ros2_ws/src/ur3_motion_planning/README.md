# UR3 Motion Planning Node

ROS 2 node that orchestrates the complete drawing pipeline: load strokes → optimize → plan with collision avoidance → execute on UR3.

---

## Build

```bash
cd ~/RS2/ros2_ws
colcon build
source install/setup.bash
```

---

## Usage

**With MoveIt2 (recommended):**
```bash
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py ur_type:=ur3
```

**Direct execution:**
```bash
ros2 run ur3_motion_planning motion_planning_node --ros-args \
  -p robot_ip:=192.168.56.101 \
  -p face:=face1
```

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | 192.168.56.101 | UR3 simulator or real robot IP |
| `robot_port` | 30002 | TCP port |
| `face` | face1 | Which face to draw (face1/face2/face3) |
| `enable_optimization` | true | Enable path optimization |

---

## Pipeline

1. **Load:** Read strokes from `{face}_strokes.json`
2. **Optimize:** Nearest-Neighbor TSP + 2-Opt (25-30% travel savings)
3. **Cartesian:** Convert pixels to robot world coordinates
4. **Plan:** MoveIt2 generates collision-safe trajectory
5. **Execute:** URScript sent directly to UR3 over TCP/IP

See code comments for implementation details.

---

## What's in `ur3_motion_planning/`

| File | Purpose |
|------|---------|
| `ur3_drawing_node.py` | ROS 2 node (main orchestrator) |
| `add_table_simple.py` | Publishes table collision object to MoveIt2 |

---

## Common Issues

**Build fails:** `source /opt/ros/humble/setup.bash && colcon clean workspace && colcon build`

**Cannot connect to robot:** Check IP is correct and port 30002 is open: `ping <ip>`

**No motion:** Ensure `add_table_simple.py` is running in another terminal (publishes MoveIt2 collision object)

**Protective stops:** Reduce motion parameters in `src/ur3_selfie_draw.py` (use Conservative level)
- Confirm UR3 is powered and in operation

**Topic subscription not receiving data**
- Verify publisher is running in separate terminal
- Check topic name: `ros2 topic list`
- Echo topic: `ros2 topic echo /stroke_paths`

**Trajectory generation times out**
- Check waypoint count isn't excessive (>3000 per stroke)
- Verify 2-Opt max iterations: 50 (configurable in ur3_selfie_draw.py)
- Reduce canvas size if needed

---

## Performance Specifications

**Trajectory Optimization:**
- Optimization reduction: 30-41% travel distance
- NN-Sort execution time: <1ms
- 2-Opt refinement time: <100ms
- Total pipeline time: 75-134ms (for typical test data)

**Generated Script Sizes:**
- Typical script: 40,000-100,000 bytes
- Maximum script: 150,000 bytes (UR3 controller limit)

**Network Performance:**
- TCP transmission rate: ~15 MB/s over Ethernet
- Typical transmission time: 50-200ms

---

## Integration Points

**Perception Subsystem Interface:**
- Subscribes to `/stroke_paths` (geometry_msgs/PoseArray)
- Expected message rate: 1-5 Hz
- Expected waypoint density: 10-100+ points per stroke

**GUI Subsystem Interface:**
- Publishes `/planning_status` (std_msgs/String)
- Status update rate: event-driven (per trajectory)
- Expected subscriber rate: 0.1-1 Hz

**UR Robot Driver Interface:**
- Publishes `/urscript_program` for manual execution
- Alternative: publishs `/joint_trajectory` for ur_robot_driver (when enabled)

---

## References

**Internal:**
- Motion Planning Library: [ur3_selfie_draw.py](../../../src/ur3_selfie_draw.py)
- Main Documentation: [README.md](../../../README.md)
- Testing Guide: [TESTING_AND_UR3_GUIDE.md](../../../TESTING_AND_UR3_GUIDE.md)

**External:**
- ROS 2 Humble: https://docs.ros.org/en/humble/
- UR Robot Documentation: https://www.universal-robots.com/articles/ur-articles/technical-specifications/
- URScript Reference: https://www.universal-robots.com/download/?option=16580

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

