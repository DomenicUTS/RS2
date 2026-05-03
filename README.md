# UR3 Selfie Drawing Robot — Motion Planning

**Team Picasso | Robotics Studio 2 | UTS**

ROS 2 motion-planning subsystem for a four-colour selfie drawing robot.
A UR3 takes strokes from the perception pipeline, plans collision-safe
trajectories with MoveIt2, and executes them on the robot via URScript.
The custom 3D-printed end-effector holds **four markers** at 0°, 90°, 180°,
and 270° — wrist_3 rotates 90° between strokes so each marker contributes
to a multi-coloured artwork.

For full system documentation (hardware setup, GUI/perception integration,
troubleshooting), see [TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md).
For cross-subsystem topic flow and run modes, see
[FULL_SYSTEM_INTEGRATION.md](FULL_SYSTEM_INTEGRATION.md).

---

## What This Subsystem Does

1. **Receives strokes** — JSON list of `[[(x,y), …], …]` on `/drawing_strokes`
   (perception subsystem) or from local `outputs/strokes/face*.json`.
2. **Optimises stroke order** — Nearest-Neighbour TSP + 2-Opt
   (~25–30% pen-up travel saved).
3. **Plans with MoveIt2** — `/compute_cartesian_path` with table + marker
   holder collision objects.
4. **Cycles markers** — Each stroke uses the next of four markers
   (wrist_3 rotates 90° during pen-up travel).
5. **Executes on the UR3** — Generates URScript and ships it over TCP
   (port 30002) to the real robot or Polyscope simulator.

---

## ⚠️ Always set `ROS_DOMAIN_ID=42` (UTS lab)

In the UTS lab every team's robot and laptop share the same network. By
default, ROS 2 nodes from different machines find each other via
multicast and end up talking to whoever happens to be online — which
means another team's perception node can publish strokes to **our**
motion node and crash a drawing mid-run, or our status updates can
flood another team's GUI.

To prevent that, **every terminal** in this project must run:

```bash
export ROS_DOMAIN_ID=42
```

…before any `ros2`, `colcon`, or `python3` command. The integrated
launch file pins the same domain via
`SetEnvironmentVariable('ROS_DOMAIN_ID', '42')`, but **external
terminals (GUI, simulator, debugging)** do not inherit that — set it
yourself. Without it, you will see strokes appear from nowhere, or your
robot freeze waiting for a topic that another team is hogging.

---

## Quick Start

```bash
# Build (after any code change)
export ROS_DOMAIN_ID=42                           # ← always first, see above
source /opt/ros/humble/setup.bash
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning
source install/setup.bash
```

**Run the integrated pipeline** (perception + motion + MoveIt2):
```bash
export ROS_DOMAIN_ID=42                           # ← always first
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui launch_rviz:=true robot_ip:=192.168.56.101
```

**Start the GUI** in a second terminal:
```bash
export ROS_DOMAIN_ID=42                           # ← always first
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py
```

For Polyscope, start the simulator first (also on the same domain):
```bash
export ROS_DOMAIN_ID=42                           # ← always first
ros2 run ur_client_library start_ursim.sh -m ur3
```

---

## Repository Layout

```
RS2/
├── README.md                       ← this file
├── FULL_SYSTEM_INTEGRATION.md      ← cross-subsystem topic map / run modes
├── TECHNICAL_DOCUMENTATION.md      ← full hardware + software documentation
├── inputs/face*.svg                ← raw drawings (test input)
├── outputs/strokes/face*.json      ← pre-baked strokes for offline runs
├── outputs/last_drawing.script     ← most recent generated URScript
├── src/
│   ├── ur3_selfie_draw.py          ← stroke optimisation + URScript constants
│   └── svg_to_json_converter.py    ← convert raw SVGs → stroke JSON
└── ros2_ws/src/ur3_motion_planning/
    ├── launch/
    │   ├── integrated_pipeline.launch.py    ← perception + MoveIt2 + motion
    │   └── ur3_motion_planning_moveit2.launch.py  ← motion only
    └── ur3_motion_planning/
        ├── ur3_drawing_node.py     ← ROS 2 node (planning + execution)
        └── add_table_simple.py     ← publishes collision scene to MoveIt2
```

---

## ROS 2 Interfaces

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Subscribe | `/drawing_strokes` | `std_msgs/String` | JSON strokes from perception |
| Subscribe | `/gui/command` | `std_msgs/String` | START / PAUSE / RESUME / STOP |
| Publish | `/drawing_status` | `std_msgs/String` | Pipeline state (LOADING, EXECUTING, COMPLETE, …) |
| Publish | `/joint_states` | `sensor_msgs/JointState` | Planned UR3 joints (10 Hz) |
| Publish | `/trajectory_preview` | `geometry_msgs/PoseArray` | RViz visualisation |

---

## Key Parameters

`ros2 run ur3_motion_planning motion_planning_node --ros-args -p ...`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | `192.168.56.101` | UR3 (real or simulator) IP |
| `robot_port` | `30002` | URScript primary interface |
| `face` | `face1` | Used when `stroke_source=file` |
| `stroke_source` | `file` | `file` (offline) or `topic` (perception) |
| `enable_optimization` | `true` | Toggle NN + 2-Opt |
| `max_step` | `0.005` | Cartesian interpolation step (m) |
| `jump_threshold` | `5.0` | MoveIt2 joint-jump filter |

Calibration constants live in `src/ur3_selfie_draw.py`:
- `CANVAS_ORIGIN_ROBOT` — canvas top-left in robot base frame
- `CANVAS_WIDTH_M`, `CANVAS_HEIGHT_M` — canvas size
- `EE_DRAW_HEIGHT`, `MARKER_TILT_DEG` — marker holder geometry
- `JOINT_VEL`, `LINEAR_VEL`, etc. — motion speeds

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Build fails | `source /opt/ros/humble/setup.bash` then rebuild |
| `ConnectionRefusedError 192.168.56.101:30002` | Start the simulator: `ros2 run ur_client_library start_ursim.sh -m ur3` |
| `move_group not available` | Wait ~25 s after launch — MoveIt2 takes a while to load |
| Robot not moving on real UR3 | Set teach pendant to **Remote Control** mode |
| Wrong colour drawn | Check the marker order in the holder matches `marker_idx = stroke % 4` |
| Strokes off-canvas | Recalibrate `CANVAS_ORIGIN_ROBOT` in `src/ur3_selfie_draw.py` |
| Strokes appear out of nowhere / robot freezes / topics from another team | `ROS_DOMAIN_ID` is wrong. **Every terminal** must `export ROS_DOMAIN_ID=42` before running anything. Verify with `echo $ROS_DOMAIN_ID`. |

---

## Standalone Test (no perception, no GUI)

```bash
export ROS_DOMAIN_ID=42                           # ← always first
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101 launch_rviz:=true

# In another terminal — load face1 and run:
export ROS_DOMAIN_ID=42                           # ← always first
ros2 run ur3_motion_planning motion_planning_node --ros-args \
  -p stroke_source:=file -p face:=face1 -p robot_ip:=192.168.56.101
```
