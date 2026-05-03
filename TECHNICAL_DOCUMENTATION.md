# UR3 Selfie Drawing Robot — Technical Documentation

**Team Picasso · Robotics Studio 2 · UTS · Sprint 4 (May 2026)**

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Key Features & Subsystems](#2-key-features--subsystems)
3. [Dependencies](#3-dependencies)
   - 3.1 [Hardware (Bill of Materials)](#31-hardware-bill-of-materials)
   - 3.2 [Computing Specs](#32-computing-specs)
   - 3.3 [Software](#33-software)
4. [Installation](#4-installation)
   - 4.1 [Hardware Setup](#41-hardware-setup)
   - 4.2 [Software Setup](#42-software-setup)
5. [Running the System](#5-running-the-system)
6. [Subsystem Reference](#6-subsystem-reference)
   - 6.1 [GUI Subsystem](#61-gui-subsystem)
   - 6.2 [Perception Subsystem](#62-perception-subsystem)
   - 6.3 [Motion Planning Subsystem](#63-motion-planning-subsystem)
7. [Configuration & Calibration](#7-configuration--calibration)
8. [Known Limitations & Assumptions](#8-known-limitations--assumptions)
9. [Troubleshooting & FAQs](#9-troubleshooting--faqs)
10. [Project Layout](#10-project-layout)

---

## 1. Project Overview

The UR3 Selfie Drawing Robot captures a selfie from a webcam and draws a
four-coloured line portrait of the subject on a fixed canvas using a
Universal Robots **UR3** with a custom 3D-printed multi-marker
end-effector. The system runs on **ROS 2 Humble** and consists of three
loosely-coupled subsystems (GUI, Perception, Motion Planning) that
communicate exclusively over ROS topics. MoveIt2 plans collision-aware
Cartesian paths around the table and the marker holder; URScript executes
the planned joint trajectories on the real robot or a Polyscope
simulator.

---

## 2. Key Features & Subsystems

| # | Subsystem | Repository | Owner | Responsibility |
|---|-----------|-----------|-------|----------------|
| 1 | **GUI** | `~/gui/` | Mateusz | Webcam capture, drawing preview, START/STOP buttons, status feedback |
| 2 | **Perception** | `~/perception/` | Nithish | Background removal → edge detection → stroke extraction |
| 3 | **Motion Planning** | `~/RS2/` | Domenic | Stroke optimisation → MoveIt2 planning → URScript execution, **4-colour marker cycling** |

**Key features:**
- Browser-quality selfie capture with PySide6 GUI (live preview, retake, draw progress).
- Robust background removal using **rembg / U²-Net** (`u2net_human_seg`).
- **Canny edge detection (σ=3)** with morphological gap-closing for clean strokes.
- **Nearest-Neighbour TSP + 2-Opt** stroke ordering (~30% pen-up travel saved).
- **MoveIt2 Cartesian planning** with table + marker-holder collision objects.
- **Four-colour drawing** via wrist_3 rotation: each stroke automatically uses the next of four markers.
- Works against the **Polyscope simulator** and on the **real UR3** with a single launch flag.

---

## 3. Dependencies

### 3.1 Hardware (Bill of Materials)

| Item | Quantity | Notes |
|------|---------:|-------|
| Universal Robots UR3 (CB-series) | 1 | The lab's UR3 |
| UR3 Teach Pendant | 1 | Required for Remote Control mode |
| 3D-printed multi-marker holder | 1 | 4 marker holes spaced 90° around the wrist axis, each tilted 20° outward; bolts to UR3 tool flange |
| Whiteboard / drawing markers | 4 | Different colours; 12 mm shaft diameter |
| Drawing canvas (whiteboard or paper) | 1 | ≥ 200 × 150 mm flat surface |
| Worktable (rigid, level) | 1 | Robot base mounts to the table edge |
| Laptop / desktop | 1 | Runs ROS 2, MoveIt2, GUI; see Computing Specs |
| USB webcam | 1 | Any V4L2-compatible camera (the laptop's built-in webcam works) |
| Ethernet cable | 1 | Robot ↔ host (or robot ↔ lab network) |

The 3D-printed marker holder is included as the source CAD/STL in the
team's shared drive. The four markers are mounted at 0°, 90°, 180°,
270° around the wrist axis, each tilted 20° outward from perpendicular
so the active tip touches the canvas at a consistent angle regardless
of which marker is selected.

### 3.2 Computing Specs

Tested and confirmed working on:
- **OS:** Ubuntu 22.04.5 LTS (Linux 6.8.0)
- **CPU:** x86-64, 4+ cores recommended (MoveIt2 planning is single-threaded but rembg uses multiple)
- **RAM:** 8 GB minimum, 16 GB recommended
- **GPU:** Optional. rembg falls back to CPU; performance is acceptable for single-image inference (~2–4 s).
- **Network:** Ethernet to robot (or routed network). Static IP routing required for the UR3 (default `192.168.0.195`; simulator default `192.168.56.101`).

### 3.3 Software

| Component | Version | Why |
|-----------|---------|-----|
| ROS 2 | **Humble Hawksbill** | All nodes are ROS 2 Humble |
| MoveIt2 | matching `ros-humble-moveit2` | `/compute_cartesian_path` and scene services |
| Universal Robots ROS 2 driver | `ros-humble-ur` | URDF, MoveIt2 config, `start_ursim.sh` |
| Python | 3.10 | Packaged with Ubuntu 22.04 |
| `rembg` (with CPU model) | latest | Background removal |
| OpenCV | ≥ 4.5 | Edge detection, contour extraction |
| NumPy | ≥ 1.20 | Coordinate math |
| PySide6 | latest | GUI |
| `cv_bridge` (ROS) | matching ROS install | `sensor_msgs/Image` ↔ OpenCV |

---

## 4. Installation

### 4.1 Hardware Setup

1. **Mount the UR3** on the rigid table; tighten the base bolts.
2. **Bolt the 3D-printed marker holder** to the UR3 tool flange. Confirm the holder is centred on the flange and that the four marker slots are oriented at 0°, 90°, 180°, 270° around the wrist axis.
3. **Insert the four markers** — the order determines the colour sequence. Stroke 1 uses marker 1 (slot at 0°), stroke 2 uses marker 2 (slot at 90°), etc.
4. **Place the canvas** flat on the table, ~200 × 150 mm of usable area, top-left corner ≈ `(x=0.185, y=0.170, z=0.010)` m in the robot base frame (this is the calibrated default; recalibrate if your table differs).
5. **Connect the Ethernet** cable between the host laptop and the UR3.
6. **Connect the webcam** (USB or built-in) to the host.

### 4.2 Software Setup

```bash
# 1. ROS 2 Humble + UR + MoveIt2 (Ubuntu 22.04)
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ur \
  ros-humble-moveit \
  ros-humble-cv-bridge \
  python3-colcon-common-extensions \
  python3-rosdep
sudo rosdep init || true
rosdep update

# 2. Python packages (used by perception + GUI)
pip3 install --user \
  "rembg[cpu]" \
  opencv-python numpy \
  PySide6

# 3. Clone the three subsystem repos into ~ (perception, gui, RS2)
#    The exact remote URLs are project-specific; clone from the team Git.
#    After cloning, the layout should be:
#       ~/perception/    ~/gui/    ~/RS2/

# 4. Build perception
export ROS_DOMAIN_ID=42                          # ← always set this, see §5
source /opt/ros/humble/setup.bash
cd ~/perception
colcon build --packages-select selfie_perception

# 5. Build motion planning
cd ~/RS2/ros2_ws
colcon build --packages-select ur3_motion_planning

# 6. Source both workspaces (add this AND `export ROS_DOMAIN_ID=42`
#    to ~/.bashrc for convenience, so every new terminal is on the
#    correct domain automatically)
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
```

If you change `setup.py` in either workspace, **clean** before
rebuilding:
```bash
rm -rf build/<pkg> install/<pkg>
colcon build --packages-select <pkg>
```

---

## 5. Running the System

The full integrated system runs in 2–3 terminals.

> ### ⚠️ Lab network safety: always `export ROS_DOMAIN_ID=42`
>
> The UTS lab puts every team's laptop and robot on the **same network**.
> Without a unique ROS 2 domain, our nodes pick up other teams' topics
> (and they pick up ours) — strokes from another group can arrive on
> `/drawing_strokes`, our `/gui/command START` can fire someone else's
> robot, and we lose runs to cross-talk. We use **`ROS_DOMAIN_ID=42`**
> as Team Picasso's reserved domain.
>
> **Every terminal listed below must begin with:**
> ```bash
> export ROS_DOMAIN_ID=42
> ```
> The integrated launch file pins the domain internally via
> `SetEnvironmentVariable('ROS_DOMAIN_ID', '42')`, but external terminals
> (GUI, simulator, `ros2 topic echo`, `colcon build`) do not inherit
> that — set it yourself. Verify with `echo $ROS_DOMAIN_ID` (must print
> `42`).

### Terminal 1 — Backend (MoveIt2 + Perception + Motion)

```bash
export ROS_DOMAIN_ID=42                          # ← always first
source /opt/ros/humble/setup.bash
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash

# For the simulator:
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  launch_rviz:=true \
  robot_ip:=192.168.56.101

# Or for the real UR3 (replace with your robot's IP):
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  launch_rviz:=true \
  robot_ip:=192.168.0.195
```

Wait ~25 s for MoveIt2 to finish loading. Look for the line
`[Scene] Marker holder attached to tool0`.

### Terminal 2 — GUI

```bash
export ROS_DOMAIN_ID=42                          # ← always first
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py
```

The PySide6 window opens with a live webcam feed.

### Terminal 3 — UR3 Polyscope Simulator (only if not using a real robot)

```bash
export ROS_DOMAIN_ID=42                          # ← always first
ros2 run ur_client_library start_ursim.sh -m ur3
```

Wait ~30 s for the Docker container to come up. Visit
`http://192.168.56.101:6080/vnc.html` to view the simulator.

### Expected outcome

1. GUI shows live webcam preview.
2. Click **Capture** → image is published on `/raw_image`.
3. Perception runs (background removal → Canny → strokes); takes 2–4 s on CPU.
4. GUI receives the stroke preview from `/drawing_preview_image` and shows the line drawing.
5. Click **Start Drawing** → motion node receives `START`, plans each stroke, executes via URScript.
6. The robot draws the portrait, **rotating wrist_3 by 90° between strokes** so each stroke is drawn with the next marker.
7. GUI status updates from `WAITING_FOR_PERCEPTION` → `OPTIMIZING_PATH` → `PLANNING_WITH_MOVEIT2` → `EXECUTING` → `COMPLETE`.

---

## 6. Subsystem Reference

### 6.1 GUI Subsystem

**Location:** `~/gui/`
**Owner:** Mateusz
**Files:**
- `selfie_drawing_gui_starter.py` — standalone PySide6 GUI (no ROS). Used for offline testing of the UI; uses simulated drawing progress.
- `selfie_drawing_gui_ros2.py` — subclass of the above that wires the buttons to ROS 2 topics. **This is the file you run for the integrated system.**

**Purpose:** Operator interface — capture a selfie, preview the drawing, start/pause/stop the robot, see live status.

**Inputs:**

| Direction | Topic | Type | Source |
|-----------|-------|------|--------|
| Subscribe | `/drawing_strokes` | `std_msgs/String` (JSON) | perception_node |
| Subscribe | `/drawing_status` | `std_msgs/String` | ur3_drawing_node |
| Subscribe | `/drawing_preview_image` | `sensor_msgs/Image` | perception_node |

**Outputs:**

| Direction | Topic | Type | Sink |
|-----------|-------|------|------|
| Publish | `/raw_image` | `sensor_msgs/Image` | perception_node |
| Publish | `/gui/command` | `std_msgs/String` | ur3_drawing_node |

Commands published on `/gui/command`: `START`, `PAUSE`, `RESUME`, `STOP`.

**How to run independently:**
```bash
# Standalone (no ROS, fake drawing — domain not relevant):
cd ~/gui && python3 selfie_drawing_gui_starter.py

# ROS 2 integrated (needs perception + motion running):
export ROS_DOMAIN_ID=42                          # ← always first
python3 ~/gui/selfie_drawing_gui_ros2.py
```

**Configurable settings:** Camera index in `CameraHandler(camera_index=N)`
(default `0`). Window size in `MainWindow.resize(1280, 780)`.

**Known limitations:**
- One webcam at a time; GUI must be restarted to switch cameras.
- No persistent settings — combo boxes (subject/style/detail) are visual only and do not affect perception parameters yet.

---

### 6.2 Perception Subsystem

**Location:** `~/perception/`
**Owner:** Nithish
**ROS 2 package:** `selfie_perception`

**Purpose:** Convert a selfie image into a list of vector strokes
(JSON) suitable for the UR3 to draw.

**Pipeline:**

```
Selfie (BGR)
   │
   ▼  rembg / U²-Net (u2net_human_seg)
Background-removed RGBA → BGR on white
   │
   ▼  Gaussian σ=3 + Canny (low=20, high=60)
Binary edge map (uint8 0/255)
   │
   ▼  Morphological closing (5×5 ellipse, 2 iter) → cv2.findContours
   ▼  Douglas-Peucker simplify (ε=3.0)
   ▼  Scale to 400×300 px canvas
   ▼  Greedy nearest-neighbour reorder
List[List[[x,y]]]  →  /drawing_strokes (JSON)
```

**Nodes (all in the `selfie_perception` package):**

| Node | Purpose | Subscribes | Publishes |
|------|---------|-----------|-----------|
| `image_loader_node` | Watches `~/perception/input/` and publishes images (file mode, no GUI) | — | `/raw_image` |
| `perception_node` | Full pipeline | `/raw_image` | `/drawing_strokes`, `/drawing_preview_image` |
| `visualization_node` | (Optional) Renders the stroke preview separately | `/drawing_strokes` | `/drawing_preview_image` |

**File outputs (all under `~/perception/output/`):**
- `perception_strokes.json` — latest stroke set (also consumed by motion as a fallback)
- `drawing_preview.png` — most recent preview render
- `run_NNN/` — per-run subfolder with `0_background_removed.png`, `1_edges.png`, `2_drawing_preview.png`

**How to run independently:**

```bash
# Standalone CLI (no ROS — domain not relevant):
cd ~/perception/src/selfie_perception
PYTHONPATH=.:$PYTHONPATH python3 selfie_perception/pipeline.py ../../input/your_selfie.png

# ROS 2 (loads from ~/perception/input/):
export ROS_DOMAIN_ID=42                          # ← always first
source ~/perception/install/setup.bash
ros2 launch selfie_perception perception_pipeline.launch.py
```

**Configurable parameters** (set via `--ros-args -p name:=value`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gaussian_sigma` | 3.0 | Pre-blur σ |
| `canny_low` / `canny_high` | 20 / 60 | Canny hysteresis thresholds |
| `simplification_epsilon` | 3.0 | Douglas-Peucker tolerance |
| `min_contour_points` | 5 | Drop contours below this length |
| `min_stroke_length` | 11 px | Drop strokes below this total length |
| `canvas_px_w`, `canvas_px_h` | 400, 300 | Output canvas |

**Known limitations & assumptions:**
- rembg only segments **human** subjects well (model: `u2net_human_seg`). Drawings of pets/objects will need a different model.
- Output is fixed to **400 × 300 pixels**; this contract is shared with the motion subsystem and changing it requires updating `CANVAS_PX_W/H` in `~/RS2/src/ur3_selfie_draw.py` too.

---

### 6.3 Motion Planning Subsystem

**Location:** `~/RS2/`
**Owner:** Domenic
**ROS 2 package:** `ur3_motion_planning`

**Purpose:** Receive strokes, plan collision-safe Cartesian paths with
MoveIt2, cycle through 4 markers, and execute on the UR3 via URScript.

**Pipeline:**

```
Strokes JSON (from /drawing_strokes or face*.json)
   │
   ▼  Scale to 95 % of canvas
   ▼  Nearest-Neighbour TSP + 2-Opt (~30 % travel saved)
Optimised stroke list
   │
   ▼  For each stroke s_idx:
   │     marker_idx = s_idx % 4
   │     quat = TOOL_QUAT ⊗ Rz(-90° × marker_idx)
   │     For travel + draw + lift:
   │        /compute_cartesian_path service (MoveIt2)  ←  COLLISION-AWARE
Joint trajectories (per stroke)
   │
   ▼  Convert to movej commands → URScript program
   ▼  TCP socket → UR3 (port 30002)
Robot draws
```

**Nodes:**

| Node | Purpose |
|------|---------|
| `motion_planning_node` (alias of `ur3_drawing_node.py`) | Main orchestrator |
| `add_table` (alias of `add_table_simple.py`) | Publishes `table` and `marker_holder` collision objects to MoveIt2 via `/apply_planning_scene` |

**Subscribes:**

| Topic | Type | Notes |
|-------|------|-------|
| `/drawing_strokes` | `std_msgs/String` (JSON) | When `stroke_source=topic` |
| `/gui/command` | `std_msgs/String` | START/PAUSE/RESUME/STOP |

**Publishes:**

| Topic | Type | Notes |
|-------|------|-------|
| `/drawing_status` | `std_msgs/String` | Pipeline state |
| `/joint_states` | `sensor_msgs/JointState` | 10 Hz, mirrors planned trajectory (RViz/MoveIt2 display sync) |
| `/trajectory_preview` | `geometry_msgs/PoseArray` | RViz only |

**Saved file:**
- `~/RS2/outputs/last_drawing.script` — the last URScript program sent to the robot. Useful for inspection and offline replay.

**How to run independently:**

```bash
# Motion-only (load face1, plan + execute on simulator):
export ROS_DOMAIN_ID=42                          # ← always first
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101 launch_rviz:=true

# Then in another terminal:
export ROS_DOMAIN_ID=42                          # ← always first
ros2 run ur3_motion_planning motion_planning_node --ros-args \
  -p stroke_source:=file -p face:=face1 -p robot_ip:=192.168.56.101
```

**Multi-marker rotation (4-colour drawing).** The 3D-printed holder
carries four markers at 0°, 90°, 180°, 270° around the wrist axis. The
node selects `marker_idx = stroke_index % 4` per stroke and rotates the
target tool orientation by `-marker_idx × 90°` around the tool's own
Z-axis. MoveIt2 plans the rest, naturally rotating wrist_3 by 90° during
the pen-up travel between strokes. Because the holder is rotationally
symmetric, every marker tip lands at the same world position when its
marker_idx is active — `CANVAS_ORIGIN_ROBOT`, `px_to_robot()`, and the
URScript `set_tcp()` value are unchanged between markers.

**Configurable parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | `192.168.56.101` | UR3 (real) or simulator IP |
| `robot_port` | `30002` | URScript primary interface |
| `stroke_source` | `file` | `file` or `topic` |
| `face` | `face1` | When loading from disk |
| `enable_optimization` | `true` | NN + 2-Opt toggle |
| `max_step` | `0.005` | MoveIt2 Cartesian interpolation (m) |
| `jump_threshold` | `5.0` | MoveIt2 joint-space jump filter |
| `planning_timeout` | `30.0` | Per-stroke planning timeout (s) |

**Calibration constants (in `~/RS2/src/ur3_selfie_draw.py`):**

| Constant | Default | Description |
|----------|---------|-------------|
| `CANVAS_ORIGIN_ROBOT` | `[0.185, 0.170, 0.010] m` | Top-left canvas corner in robot base frame |
| `CANVAS_WIDTH_M`, `CANVAS_HEIGHT_M` | `0.150 m, 0.120 m` | Canvas size |
| `EE_DRAW_HEIGHT` | `0.115 m` | EE height above canvas surface |
| `MARKER_TILT_DEG` | `20°` | Marker tilt from perpendicular |
| `JOINT_VEL`, `LINEAR_VEL`, etc. | mid-speed | URScript motion params |

**Known limitations & assumptions:**
- The 4 markers must be physically loaded into slots 0, 90, 180, 270° in that order — the colour sequence per stroke is determined by slot index.
- The holder's collision geometry is approximated as a single 160 × 180 mm box. This is conservative for any single marker but does not exactly match the 4-radial-arms shape.
- URScript only contains `movej` commands (joint-space moves). MoveIt2 plans Cartesian paths and we feed the resulting joint waypoints directly. `set_tcp()` is set as a courtesy but does not affect `movej` motion.
- Collision avoidance is enforced **at planning time**. URScript execution itself is open-loop; if the robot is moved by hand mid-execution, no replanning happens.

---

## 7. Configuration & Calibration

### Canvas calibration (most common change)

If the canvas position or size differs from the default, edit
`~/RS2/src/ur3_selfie_draw.py` lines ~36–38:

```python
CANVAS_ORIGIN_ROBOT = np.array([0.185, 0.170, 0.010])  # m, robot base frame
CANVAS_WIDTH_M      = 0.150                              # 15 cm
CANVAS_HEIGHT_M     = 0.120                              # 12 cm
```

To find these values: jog the robot tip to the **top-left** corner of
the canvas (with marker 1 active), record the X/Y/Z reading from the
teach pendant, and update the constant. Width and height are the
physical canvas extents.

After editing, rebuild:
```bash
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning
```

### Motion speeds

Three predefined levels in `ur3_selfie_draw.py`:
- **Conservative** (real robot first runs): `(0.73, 0.80, 0.47, 0.10)`
- **Mid** (current default): `(0.97, 1.10, 0.64, 0.15)`
- **Maximum** (simulator only): `(1.20, 1.40, 0.80, 0.20)`

Tuple is `(JOINT_ACCEL, JOINT_VEL, LINEAR_ACCEL, LINEAR_VEL)`.

### Marker holder geometry

Edit `EE_DRAW_HEIGHT` (length from flange to marker tip along the
tilted axis) and `MARKER_TILT_DEG` if you 3D-print a different holder.
The collision-object dimensions are also defined in
`add_table_simple.py` and `ur3_drawing_node.py::_publish_scene_objects`
— update them together.

---

## 8. Known Limitations & Assumptions

1. **Rotationally-symmetric holder required.** The 4-marker cycling
   relies on the holder being rotationally symmetric so every marker
   tip lands at the same world position. A non-symmetric holder would
   need per-marker `set_tcp()` updates and a per-marker
   `CANVAS_ORIGIN_ROBOT`.
2. **400 × 300 px canvas contract.** This is hard-coded across all
   three subsystems. Changing it requires synchronised edits to
   `stroke_extraction.py` (perception) and `ur3_selfie_draw.py`
   (motion).
3. **rembg = humans only.** Subjects without a clear human silhouette
   will not segment well.
4. **Single-shot pipeline.** The motion node assumes one stroke set
   per drawing. Re-capturing in the GUI will trigger a new run, but
   only after the previous one reaches `COMPLETE` or `ERROR`.
5. **Open-loop URScript execution.** No mid-stroke replanning if the
   environment changes.
6. **No tool-changer.** Markers are physically swapped only via
   wrist_3 rotation between strokes. The colour order is determined by
   how you load the holder.

---

## 9. Troubleshooting & FAQs

| Symptom | Likely cause / fix |
|---------|--------------------|
| Strokes from another team appear / our START fires another team's robot / topics flicker between values | **`ROS_DOMAIN_ID` mismatch.** In the UTS lab everyone shares one network. Every terminal must `export ROS_DOMAIN_ID=42` before any `ros2`/`colcon`/`python3` command. Verify with `echo $ROS_DOMAIN_ID` (must print `42`). Add to `~/.bashrc` to make it sticky. |
| GUI never receives the perception preview / motion never receives strokes | Same domain mismatch as above — usually one terminal is missing `export ROS_DOMAIN_ID=42`. The integrated launch file sets the domain *only inside its own process tree*; external terminals must set it manually. |
| Build fails with “colcon: command not found” | Source ROS 2: `source /opt/ros/humble/setup.bash` |
| `move_group` not available; `/compute_cartesian_path` unavailable | MoveIt2 still loading. Wait ≥ 25 s after launching. The launch file already includes a `TimerAction` delay for the scene + motion nodes, but the GUI / external triggers may need to wait too. |
| `ConnectionRefusedError 192.168.56.101:30002` | Polyscope simulator isn't running. Start it: `ros2 run ur_client_library start_ursim.sh -m ur3`. |
| Robot is connected but does nothing | Real UR3: pendant must be in **Remote Control** mode (Settings → System → Remote Control). |
| Robot makes a protective stop on first move | Reduce motion params to **Conservative** (see §7). Verify `CANVAS_ORIGIN_ROBOT` is reachable. |
| Strokes are drawn off the canvas | Recalibrate `CANVAS_ORIGIN_ROBOT` (see §7). |
| Wrong colour for a stroke | The colour order is determined by how you physically load markers into the holder slots. Stroke `i` uses slot `i % 4`. Re-arrange the markers, no code change needed. |
| GUI doesn't see the perception preview | Confirm `ROS_DOMAIN_ID=42` is set in the GUI terminal *and* the launch file does `SetEnvironmentVariable('ROS_DOMAIN_ID', '42')`. Both must match. |
| GUI camera shows "Disconnected" | Camera index conflict; close other apps using the webcam, or change `CameraHandler(camera_index=1)`. |
| `rembg` slow on first run | Model download (~170 MB). Subsequent runs are fast (~2–4 s on CPU). |
| Phantom robot in RViz | Only one source should publish `/joint_states`. The motion node does this — make sure no `joint_state_publisher` is launched separately. |
| RViz shows red collision flash | A planned waypoint touches the table or holder. Check `add_table` ran successfully (look for `Table collision object applied`). |
| `/compute_cartesian_path fraction < 0.5` | Travel pose unreachable. Verify the canvas calibration; reduce `max_step` (smaller steps allow MoveIt2 more flexibility). |

**FAQ — How do I change the colour cycle order?**
Re-order the markers in the physical holder. The code always uses
`stroke_index % 4`, mapping strokes to slots in slot-index order.

**FAQ — How do I draw with a single colour?**
Either load the same colour into all four slots, or short-circuit the
multi-marker code: in `ur3_drawing_node.py::_plan_and_build_urscript`,
hard-code `marker_idx = 0` for every stroke.

**FAQ — Where is the last URScript saved?**
`~/RS2/outputs/last_drawing.script`. You can replay it manually via
the simulator/robot (`socket.connect((ip, 30002)); sock.send(script)`).

---

## 10. Project Layout

```
~/RS2/                                       ← Motion planning + integration docs
├── README.md
├── FULL_SYSTEM_INTEGRATION.md
├── TECHNICAL_DOCUMENTATION.md               ← (this file)
├── inputs/face*.svg
├── outputs/
│   ├── strokes/face*.json
│   ├── verified/face*.svg
│   └── last_drawing.script
├── src/
│   ├── ur3_selfie_draw.py
│   └── svg_to_json_converter.py
└── ros2_ws/src/ur3_motion_planning/
    ├── launch/
    │   ├── integrated_pipeline.launch.py
    │   └── ur3_motion_planning_moveit2.launch.py
    └── ur3_motion_planning/
        ├── ur3_drawing_node.py
        └── add_table_simple.py

~/perception/                                ← Perception subsystem
├── README.md
├── input/
├── output/
└── src/selfie_perception/
    ├── launch/perception_pipeline.launch.py
    └── selfie_perception/
        ├── background_removal.py
        ├── edge_detection.py
        ├── stroke_extraction.py
        ├── pipeline.py             ← standalone CLI
        ├── perception_node.py
        ├── image_loader_node.py
        └── visualization_node.py

~/gui/                                       ← GUI subsystem
├── readme.txt
├── selfie_drawing_gui_starter.py
└── selfie_drawing_gui_ros2.py
```

---

**Maintainers:** Team Picasso (Mateusz, Nithish, Domenic).
**Source:** [RS2 GitHub repository](https://github.com/) (fill in your repo URL).
**Last verified:** 3 May 2026 against ROS 2 Humble + MoveIt2 + UR ROS 2 driver.
