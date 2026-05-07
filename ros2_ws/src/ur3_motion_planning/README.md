# `ur3_motion_planning` — ROS 2 Package

Motion-planning node for the UR3 Selfie Drawing Robot (Team Picasso).
Subscribes to perception strokes and a colour selection from the GUI,
plans collision-aware Cartesian paths with MoveIt2, rotates wrist_3 to
the chosen marker, and executes URScript on the UR3.

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

## Colour Selection (Single-Marker Drawing)

The end-effector holder carries 4 markers spaced 90° around the wrist_3
axis, each tilted 20° outward. The GUI sends `START:<colour>` on
`/gui/command` (e.g. `START:blue`); the motion node parses the colour
out of the start command, looks it up in `COLOUR_TO_MARKER` to get a
slot index, and uses that one marker for the whole drawing.

**How the wrist is rotated to the chosen marker:**

1. The trajectory is planned for **every** stroke with marker 1's
   orientation (`TOOL_QUAT`). This makes the plan deterministic — same
   tool0 path regardless of colour.
2. After all planning is done, the 6th joint (`wrist_3`) of every
   output waypoint is shifted by `marker_idx * -90°` (with `±2π` wrap).
3. A "rotate-only" `movej` is prepended to the URScript so the wrist
   visibly swaps to the chosen marker right after HOME, before any
   horizontal motion toward the canvas.

Why post-process rather than ask MoveIt2 for the rotated orientation
directly: on a 6-DOF UR3 the same end-effector orientation is reachable
through multiple IK branches, and `/compute_cartesian_path` was free
to "absorb" the requested 90° tool-Z rotation into wrist_1 / wrist_2 /
a wrist-flip — leaving wrist_3 unchanged regardless of colour. The
post-processing offset on wrist_3 sidesteps the IK ambiguity entirely.
Because the holder is rotationally symmetric, the constant offset on
wrist_3 swings the chosen marker into the position marker 1 was
tracing without changing the marker tip's world coordinates.

The motion node also intentionally does **not** auto-start drawing
when perception strokes arrive on `/drawing_strokes` — it caches them
and waits for the GUI's `START:<colour>`. (Auto-start was the previous
behaviour and caused every drawing to come out in the default colour.)

See [`ur3_drawing_node.py`](ur3_motion_planning/ur3_drawing_node.py) —
look for `COLOUR_TO_MARKER`, `marker_tool_quat()`, `_on_gui_command()`,
and the post-processing block at the end of `_plan_and_build_urscript()`.

To change the colour-to-slot mapping (e.g. you loaded the markers in a
different order), edit `COLOUR_TO_MARKER` directly:

```python
COLOUR_TO_MARKER = {
    "red":   0,
    "blue":  1,
    "green": 2,
    "black": 3,
}
```

**Verifying the data flow.** When you click Start Drawing in the GUI,
the motion node should print these four log lines in order. If any is
missing or shows the wrong colour, that's where the chain is broken.

```
[GUI] >>> Received raw command: 'START:blue'
[GUI] Parsed command='START' payload='blue'
[GUI] ✓ Colour set to 'blue' (marker slot 2/4)
[Plan] >>> Pipeline reading colour: 'blue' (marker_idx=1, wrist_3_offset=-90.0°)
```

---

## ROS 2 Topics

| Direction | Topic | Type | Used by |
|-----------|-------|------|---------|
| Subscribe | `/drawing_strokes` | `std_msgs/String` | Perception → motion (cached, does **not** auto-start the pipeline) |
| Subscribe | `/gui/command` | `std_msgs/String` | GUI → motion. `START:<colour>` (the trigger), `PAUSE`, `RESUME`, `STOP` |
| Subscribe | `/gui/marker_colour` | `std_msgs/String` | GUI → motion (`red`/`blue`/`green`/`black`). Supplementary — the authoritative colour comes from `START:<colour>`. |
| Publish | `/drawing_status` | `std_msgs/String` | Motion → GUI |
| Publish | `/trajectory_preview` | `geometry_msgs/PoseArray` | RViz |
| Publish | `/joint_states` | `sensor_msgs/JointState` | MoveIt2 / RViz |

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| Topics from / to another team appear (or our `START` triggers their robot) | `ROS_DOMAIN_ID` mismatch. Every terminal must `export ROS_DOMAIN_ID=42` before any `ros2`/`colcon`/`python3` call. Verify with `echo $ROS_DOMAIN_ID`. |
| Drawing always uses the default marker regardless of GUI selection | Watch the motion-node log when you click Start. The chain is: `[GUI] >>> Received raw command: 'START:<colour>'` → `[GUI] Parsed command='START' payload='<colour>'` → `[GUI] ✓ Colour set to '<colour>'` → `[Plan] >>> Pipeline reading colour: '<colour>'`. If the first line never appears, the GUI message isn't reaching the node (check `ROS_DOMAIN_ID`). If a later line shows the wrong colour, the bug is in that step. |
| Pipeline never starts after Process | Expected — auto-start was removed deliberately. Click **Start Drawing** in the GUI to trigger drawing. |
| `move_group not available` | MoveIt2 still loading. Wait ≥ 25 s after launch, or use the built-in `TimerAction` delays. |
| `/compute_cartesian_path` timeout | Stroke too long — reduce points; or `jump_threshold` too tight |
| `Cartesian plan fraction < 0.5` for travel | Travel pose unreachable; check `CANVAS_ORIGIN_ROBOT` calibration |
| Robot draws nothing on real UR3 | Pendant must be in **Remote Control** mode; check protective stop |
| Phantom robot in RViz | The drawing node publishes `/joint_states` itself — ensure no other `joint_state_publisher` is running |
