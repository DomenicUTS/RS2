# `ur3_motion_planning` — ROS 2 Package

Motion-planning node for the UR3 Selfie Drawing Robot (Team Picasso).
Subscribes to perception strokes, plans collision-aware Cartesian paths
with MoveIt2, cycles through 4 markers (one colour per stroke), and
executes URScript on the UR3.

For top-level setup and the integrated pipeline, see
[`~/RS2/README.md`](../../../README.md) and
[`~/RS2/FULL_SYSTEM_INTEGRATION.md`](../../../FULL_SYSTEM_INTEGRATION.md).

---

## ⚠️ `ROS_DOMAIN_ID=42` is mandatory in the UTS lab

Every terminal that runs anything from this package must first do:

```bash
export ROS_DOMAIN_ID=42
```

The UTS lab shares one network across all teams. Without a unique domain,
ROS 2 nodes from other teams pick up our topics (and vice versa) — strokes
appear from nowhere, START commands fire someone else's robot, etc. The
integrated launch file pins this internally, but external terminals (your
own debugging, `ros2 topic`, the GUI, the simulator) do not inherit it.
Set it manually, every time.

---

## Build

```bash
export ROS_DOMAIN_ID=42                          # ← always first
source /opt/ros/humble/setup.bash
cd ~/RS2/ros2_ws
colcon build --packages-select ur3_motion_planning
source install/setup.bash
```

If you change `setup.py` entry points, clean first:
```bash
rm -rf build/ur3_motion_planning install/ur3_motion_planning
colcon build --packages-select ur3_motion_planning
```

---

## Launch Files

| File | Purpose |
|------|---------|
| `integrated_pipeline.launch.py` | Full backend: MoveIt2 + perception + motion. Use with the GUI. |
| `ur3_motion_planning_moveit2.launch.py` | Motion-only: MoveIt2 + scene + `motion_planning_node`. For offline / file-based runs. |

```bash
# Integrated (with GUI in another terminal):
export ROS_DOMAIN_ID=42                          # ← always first
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui launch_rviz:=true robot_ip:=192.168.56.101

# Standalone (loads ~/RS2/outputs/strokes/face1_strokes.json):
export ROS_DOMAIN_ID=42                          # ← always first
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101
```

---

## Node Parameters (`motion_planning_node`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | `192.168.56.101` | UR3 (real or simulator) IP |
| `robot_port` | `30002` | URScript primary interface |
| `stroke_source` | `file` | `file` or `topic` |
| `face` | `face1` | Local stroke file (when `stroke_source=file`) |
| `enable_optimization` | `true` | Toggle NN + 2-Opt |
| `max_step` | `0.005` | MoveIt2 Cartesian interpolation step (m) |
| `jump_threshold` | `5.0` | MoveIt2 joint-space jump filter |
| `planning_timeout` | `30.0` | Per-stroke planning timeout (s) |

---

## Files

| File | Purpose |
|------|---------|
| `ur3_motion_planning/ur3_drawing_node.py` | Main ROS 2 node — load → optimise → MoveIt2 plan → URScript execute |
| `ur3_motion_planning/add_table_simple.py` | Publishes table + marker holder collision objects via `/apply_planning_scene` |
| `launch/integrated_pipeline.launch.py` | Backend launch (MoveIt2 + perception + motion + scene) |
| `launch/ur3_motion_planning_moveit2.launch.py` | Motion-only launch |
| `setup.py` / `setup.cfg` / `package.xml` | ROS 2 package metadata |

---

## Multi-Marker Drawing

The end-effector holder carries 4 markers spaced 90° around the wrist_3 axis,
each tilted 20° outward. Per stroke, the node selects the next marker
(`marker_idx = stroke_index % 4`) and rotates the planned tool orientation
about its own Z by `-marker_idx * 90°`. MoveIt2 then naturally rotates
wrist_3 by 90° during the pen-up travel between strokes, swapping the
active marker. Because the holder is rotationally symmetric, every marker
tip lands at the same world position — `px_to_robot()` and `set_tcp()` do
not change between markers.

See [`ur3_drawing_node.py`](ur3_motion_planning/ur3_drawing_node.py) —
look for `marker_tool_quat()` and the per-stroke loop in
`_plan_and_build_urscript()`.

---

## ROS 2 Topics

| Direction | Topic | Type | Used by |
|-----------|-------|------|---------|
| Subscribe | `/drawing_strokes` | `std_msgs/String` | Perception → motion |
| Subscribe | `/gui/command` | `std_msgs/String` | GUI → motion (START/STOP/…) |
| Publish | `/drawing_status` | `std_msgs/String` | Motion → GUI |
| Publish | `/trajectory_preview` | `geometry_msgs/PoseArray` | RViz |
| Publish | `/joint_states` | `sensor_msgs/JointState` | MoveIt2 / RViz |

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| Topics from / to another team appear (or our `START` triggers their robot) | `ROS_DOMAIN_ID` mismatch. Every terminal must `export ROS_DOMAIN_ID=42` before any `ros2`/`colcon`/`python3` call. Verify with `echo $ROS_DOMAIN_ID`. |
| `move_group not available` | MoveIt2 still loading. Wait ≥ 25 s after launch, or use the built-in `TimerAction` delays. |
| `/compute_cartesian_path` timeout | Stroke too long — reduce points; or `jump_threshold` too tight |
| `Cartesian plan fraction < 0.5` for travel | Travel pose unreachable; check `CANVAS_ORIGIN_ROBOT` calibration |
| Robot draws nothing on real UR3 | Pendant must be in **Remote Control** mode; check protective stop |
| Phantom robot in RViz | The drawing node publishes `/joint_states` itself — ensure no other `joint_state_publisher` is running |
