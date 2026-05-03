# Full System Integration — GUI + Perception + Motion Planning

**Project:** UR3 Selfie Drawing Robot — Team Picasso
**Last updated:** 3 May 2026
**Contributors:** GUI student, Perception student, Domenic Kadioglu (Motion)

> See also: [TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md) for full
> hardware/software bill of materials, install steps, and per-subsystem
> reference. This document is the cross-system topic map and run-mode guide.

---

## ⚠️ Always set `ROS_DOMAIN_ID=42` (UTS lab)

In the UTS lab every team's robot and laptop are on the same network.
By default, ROS 2 nodes from different machines find each other via
multicast and end up cross-talking — another team's perception node
can publish strokes to **our** motion node, or our `/gui/command`
START messages can fire someone else's robot. We've had drawings get
corrupted by stray topics in past sprints; this is the fix.

**Every terminal that runs `ros2`, `colcon`, or any of our Python nodes
must first `export ROS_DOMAIN_ID=42`.** The `integrated_pipeline.launch.py`
file pins the same domain internally via
`SetEnvironmentVariable('ROS_DOMAIN_ID', '42')`, but external terminals
(GUI, simulator, monitoring with `ros2 topic`) do not inherit that —
you have to set it yourself.

```bash
export ROS_DOMAIN_ID=42      # do this in every terminal, before any ros2/colcon command
echo $ROS_DOMAIN_ID          # verify (must print: 42)
```

If different terminals have different domain IDs, nodes will not see
each other and the pipeline will silently hang (e.g. the GUI never
shows the preview, or the motion node never receives `/drawing_strokes`).

---

## Prerequisites & Setup

**Important:** After any code changes, you must rebuild the affected packages before running:

```bash
# For motion planning changes:
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# For perception changes:
cd ~/perception && colcon build --packages-select selfie_perception

# For both (when changes span subsystems):
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning
```

**Key files for ROS 2 Python packages:**
- `setup.py` — Entry points for `ros2 run`
- `setup.cfg` — (NEW) Tells colcon to install executables to `lib/<pkg>/` for `ros2 run` to find them
- `package.xml` — Package dependencies and metadata

If you add or modify entry points in `setup.py`, you may need to clean the build cache:
```bash
# Motion planning:
rm -rf ~/RS2/ros2_ws/build/ur3_motion_planning ~/RS2/ros2_ws/install/ur3_motion_planning
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Perception:
rm -rf ~/perception/build/selfie_perception ~/perception/install/selfie_perception
cd ~/perception && colcon build --packages-select selfie_perception
```

---

## System Overview

Three independent subsystems work together to capture a selfie and draw it with a UR3 robot arm:

```
┌──────────────┐       /raw_image        ┌──────────────────┐     /drawing_strokes    ┌──────────────────┐
│              │  ───────────────────────→│                  │  ───────────────────────→│                  │
│     GUI      │                          │    PERCEPTION    │                          │  MOTION PLANNING │
│  (PySide6)   │  ←───────────────────── │  (selfie_perc.)  │  ←───────────────────── │  (ur3_motion)    │
│              │  /drawing_strokes        │                  │  /drawing_status         │                  │
│              │  /drawing_preview_image  │                  │                          │                  │
│              │  /drawing_status         │                  │                          │                  │
│              │  ───────────────────────→│                  │                          │                  │
│              │       /gui/command       │                  │  ───────────────────────→│                  │
└──────────────┘                          └──────────────────┘       /gui/command       └──────────────────┘
   ~/gui/                                    ~/perception/                                 ~/RS2/
```

---

## ROS 2 Topics — Complete Map

| Topic | Message Type | Publisher(s) | Subscriber(s) | Description |
|-------|-------------|-------------|---------------|-------------|
| `/raw_image` | `sensor_msgs/Image` | **GUI** node or **image_loader_node** | perception_node, visualization_node (reset) | Raw camera frame (BGR8) |
| `/drawing_strokes` | `std_msgs/String` | perception_node | **ur3_drawing_node**, **GUI** node, visualization_node | JSON stroke array `[[[x,y],…],…]` in 400×300 px |
| `/drawing_preview_image` | `sensor_msgs/Image` | perception_node, visualization_node | **GUI** node | Rendered preview of what robot will draw |
| `/drawing_status` | `std_msgs/String` | **ur3_drawing_node** | **GUI** node | Pipeline state (WAITING, EXECUTING, COMPLETE, ERROR, …) |
| `/trajectory_preview` | `geometry_msgs/PoseArray` | ur3_drawing_node | RViz | 3D waypoint visualisation |
| `/gui/command` | `std_msgs/String` | **GUI** node | **ur3_drawing_node** | Commands: START, PAUSE, RESUME, STOP |

---

## Each Module's Inputs and Outputs

### GUI (`~/gui/`)

| Direction | Topic | What |
|-----------|-------|------|
| **Publishes** | `/raw_image` | Captured photo from laptop webcam |
| **Publishes** | `/gui/command` | START / PAUSE / RESUME / STOP commands |
| **Subscribes** | `/drawing_strokes` | Receives stroke JSON → renders preview |
| **Subscribes** | `/drawing_status` | Progress/state updates from motion |
| **Subscribes** | `/drawing_preview_image` | Preview image from perception visualisation |

**Files:**
- `selfie_drawing_gui_starter.py` — Original standalone GUI (no ROS, fake drawing)
- `selfie_drawing_gui_ros2.py` — ROS 2 integrated GUI (inherits from starter)

### Perception (`~/perception/`)

| Direction | Topic | What |
|-----------|-------|------|
| **Subscribes** | `/raw_image` | Input photo (from GUI or image_loader_node) |
| **Publishes** | `/drawing_strokes` | JSON strokes after bg removal + edge detection + stroke extraction |
| **Publishes** | `/drawing_preview_image` | Rendered preview of strokes |
| **File output** | `~/perception/output/perception_strokes.json` | Same strokes saved to disk |

**Nodes:**
- `image_loader_node` — Watches `~/perception/input/` for images (standalone testing, publishes up to 5 times)
- `perception_node` — Full pipeline: background removal (rembg/u2net_human_seg) → Gaussian blur + Canny edge detection (σ=3) → contour extraction, Douglas-Peucker simplification, morphological closing, nearest-neighbour stroke ordering. Publishes strokes and preview.
- `visualization_node` — Subscribes to `/drawing_strokes`, renders stroke preview, publishes to `/drawing_preview_image`, saves `drawing_preview.png`
- `pipeline` — Standalone CLI tool (no ROS) for running the full pipeline on an image file

### Motion Planning (`~/RS2/`)

| Direction | Topic | What |
|-----------|-------|------|
| **Subscribes** | `/drawing_strokes` | Receives strokes from perception |
| **Subscribes** | `/gui/command` | START / PAUSE / RESUME / STOP from GUI |
| **Publishes** | `/drawing_status` | Pipeline state updates |
| **Publishes** | `/trajectory_preview` | PoseArray for RViz |
| **File input** | `~/RS2/outputs/strokes/face*_strokes.json` | Pre-made strokes (standalone mode) |

**Nodes:**
- `ur3_drawing_node` — Full pipeline: load → optimise → plan → execute

---

## How to Run

### Mode 1: Full Integration (GUI + Perception + Motion with MoveIt2)

**Simulator (default, with MoveIt2 collision planning):**
```bash
# Build both packages (required after any code changes)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Terminal 1 — Start the backend pipeline (MoveIt2 + Perception + Motion)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  launch_rviz:=true

# Terminal 2 — Start the GUI
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py

# Terminal 3 (optional) — Start the UR3 simulator
export ROS_DOMAIN_ID=42
ros2 run ur_client_library start_ursim.sh -m ur3
```

**Real Robot (replace IP with your UR3 address):**
```bash
# Build both packages (required after any code changes)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Terminal 1 — Start the backend pipeline
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  robot_ip:=192.168.0.195 \
  launch_rviz:=true

# Terminal 2 — Start the GUI
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py
```

**Workflow:**
1. GUI shows live webcam preview
2. Click "Capture" → sends image to perception
3. Perception processes: background removal (rembg) → Canny edge detection (σ=3) → stroke extraction → publishes strokes
4. GUI shows preview of what will be drawn
5. Click "Start Drawing" button
6. Motion node receives strokes, plans collision-safe MoveIt2 trajectories, executes via URScript
7. Robot draws the face at 20° angle, **cycling through 4 markers** — wrist_3 rotates 90° between strokes so each stroke gets the next colour, producing a four-coloured artwork

### Mode 2: Perception + Motion with MoveIt2 (no GUI)

```bash
# Build both packages (required after any code changes)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Place an image in ~/perception/input/
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=file \
  launch_rviz:=true
```

The `image_loader_node` will automatically load and publish images, then perception_node processes → strokes published → motion node plans and executes.

### Mode 3: Each Subsystem Standalone

**GUI only** (fake processing, no ROS needed):
```bash
cd ~/gui
python3 selfie_drawing_gui_starter.py
```

**Perception only** (process an image, no robot):
```bash
# Via ROS 2 launch (place image in ~/perception/input/ first):
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
ros2 launch selfie_perception perception_pipeline.launch.py

# OR standalone (no ROS at all):
python3 -m selfie_perception.pipeline ~/perception/input/your_image.png
# OR auto-pick latest image:
python3 -m selfie_perception.pipeline
```

**Motion only (MoveIt2 planning from pre-made strokes):**
```bash
# Build the motion planning package
source ~/RS2/ros2_ws/install/setup.bash
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Terminal 1 — Start MoveIt2 with scene setup
export ROS_DOMAIN_ID=42
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101 \
  launch_rviz:=true

# Terminal 2 — Start the motion node in file mode:
export ROS_DOMAIN_ID=42
source ~/RS2/ros2_ws/install/setup.bash
ros2 run ur3_motion_planning motion_planning_node \
  --ros-args \
  -p stroke_source:=file \
  -p face:=face1 \
  -p robot_ip:=192.168.56.101
```

This will load `~/RS2/outputs/strokes/face1_strokes.json`, plan with MoveIt2 collision avoidance, and execute.

---

## Data Format Agreement

All three subsystems agree on this stroke format:

```json
[
  [[x1, y1], [x2, y2], ...],   // stroke 1
  [[x1, y1], [x2, y2], ...],   // stroke 2
  ...
]
```

- **Canvas:** 400 × 300 pixels
- **Origin:** top-left, Y increases downward
- **Coordinates:** float, 1 decimal place
- **Transported via:** `/drawing_strokes` topic (std_msgs/String with JSON) or file

---

## Changes Made for Integration

### GUI (`~/gui/`)

| File | Change |
|------|--------|
| `selfie_drawing_gui_ros2.py` | **NEW** — ROS 2 integrated GUI subclass. Publishes captures to `/raw_image`, subscribes to `/drawing_strokes` + `/drawing_status` + `/drawing_preview_image`, sends commands on `/gui/command`. |
| `selfie_drawing_gui_starter.py` | **Unchanged** — still works standalone without ROS. |

### Perception (`~/perception/src/selfie_perception/selfie_perception/`)

| File | Change |
|------|--------|
| `perception_node.py` | **NEW** — Single unified perception node. Subscribes to `/raw_image`, runs bg removal (rembg/u2net_human_seg) → Canny edge detection (σ=3) → stroke extraction (morphological closing + Douglas-Peucker simplification + NN reordering). Publishes `/drawing_strokes` and `/drawing_preview_image`. Replaces the old 3-node pipeline (face_detection + image_processing + mapping). |
| `background_removal.py` | **NEW** — Background removal module using `rembg` library with u2net_human_seg model. |
| `edge_detection.py` | **NEW** — Canny edge detection with configurable Gaussian pre-blur (σ=3 default). |
| `stroke_extraction.py` | **NEW** — Contour extraction, Douglas-Peucker simplification, morphological closing, greedy NN stroke reordering. |
| `pipeline.py` | **NEW** — Standalone CLI pipeline (no ROS). Runs all 3 stages and saves outputs to numbered run directories. |
| `image_loader_node.py` | Watches `~/perception/input/` for images, publishes up to `max_publishes` times (default 5). |
| `visualization_node.py` | Subscribes to `/drawing_strokes`, renders preview, publishes to `/drawing_preview_image`, saves to disk. Resets on new `/raw_image`. |
| `perception_pipeline.launch.py` | Updated to launch `image_loader_node` → `perception_node` → `visualization_node`. |

### Motion Planning (`~/RS2/`)

| File | Change |
|------|--------|
| `ur3_drawing_node.py` | **REWRITTEN** — Now uses MoveIt2 `/compute_cartesian_path` service for collision-aware trajectory planning. Calls GetCartesianPath for each stroke, then converts planned joint trajectories to URScript. |
| `add_table_simple.py` | **REWRITTEN** — Publishes table AND marker-holder (160×180mm prism) as collision objects. Marker holder is **attached to tool0** so it moves with EE and MoveIt2 knows to avoid collisions. |
| `integrated_pipeline.launch.py` | Includes `ur_moveit_config` launch. Adds delays and TimerActions to: (1) start move_group, (2) publish scene objects, (3) start drawing node. Launches `perception_node` + `visualization_node` from selfie_perception. |
| `ur3_motion_planning_moveit2.launch.py` | Updated to include scene setup (table + marker holder auto-published). |
| `setup.py` | Added `add_table` console script entry point. |
| `setup.cfg` | **NEW** — Required for ROS 2 Python packages. Tells `colcon` to install entry point executables to `lib/ur3_motion_planning/` instead of `bin/` (needed for `ros2 run`). |
| `ur3_selfie_draw.py` | No changes (20° tilt + 15cm EE height already configured). |

---

## Architecture Principles

1. **Loose coupling via topics** — each module only knows about topic names and message types, not about each other's internals.
2. **Standalone first** — every module can run and be tested independently. Integration is additive.
3. **Single source of truth** — stroke format is defined once (400×300 px JSON). No translation layers needed.
4. **Reset-capable** — perception and motion nodes can process multiple images in a session (the GUI can retake and re-process).

---

## MoveIt2 Trajectory Planning & Tilted Tool Drawing

### Architecture: MoveIt2 Planning → URScript Execution

The motion planning node now **plans with MoveIt2 but executes with URScript** for maximum compatibility:

```
Strokes (JSON)
     ↓
Optimize (NN + 2-Opt)
     ↓
Convert to Cartesian waypoints (xyz positions)
     ↓
Call /compute_cartesian_path (MoveIt2) ← COLLISION-AWARE
     ↓
Get planned joint-space trajectory
     ↓
Convert to URScript (movej commands)
     ↓
Send via TCP socket → Robot (simulator or real)
```

### Marker Holder Geometry & Tool Orientation

The marker is held in a **3D-printed angled holder** that:
- Is bolted to the UR3 tool0 flange
- Extends the marker tip **15 cm below the end effector** (0.150 m)
- Is tilted **20° from perpendicular** (toward the camera)

This creates a **TCP (Tool Center Point) offset** that must be known to both:
1. **MoveIt2** — so it plans paths for the marker tip, not the bare flange
2. **URScript** — via the `set_tcp()` command at the start of each program

**Offset values** (in robot base frame, in meters):
```
TCP_X  = 0.0513 m  (forward, due to 20° tilt)
TCP_Y  = 0.0 m     (centered)
TCP_Z  = -0.1410 m (downward, 15 cm + geometry)
Rotation = [20° around Y-axis]
```

**How MoveIt2 knows about the offset:**
- The drawing node specifies `link_name = "tool0"` when calling `/compute_cartesian_path`
- The marker holder is published as an **AttachedCollisionObject** linked to `tool0`
  ```python
  attached = AttachedCollisionObject()
  attached.link_name = "tool0"
  attached.object = marker_holder_box
  attached.touch_links = ["tool0", "wrist_3_link"]  # Don't self-collide
  ```
- MoveIt2 carries this collision shape with the end effector during planning, so paths avoid hitting objects with the marker

**How URScript applies the offset:**
At the start of every URScript program:
```
def draw_face():
  set_tcp(p[0.0513, 0.0, -0.1410, 0.0, 0.3491, 0.0])
  # ... rest of program ...
end
```

This tells the robot: *"Treat this offset as the tool center for all movel/movej paths."* The robot's IK solver then adjusts joint angles so the actual marker tip reaches the commanded positions.

### Multi-Marker (4-Colour) Drawing

The 3D-printed end-effector holder carries **four markers** at 0°, 90°, 180°,
and 270° around the wrist_3 axis, each tilted 20° outward (radially).
Because the holder is rotationally symmetric, when wrist_3 rotates 90°
the next marker tip lands at the same world position the previous one
occupied — so the canvas calibration (`CANVAS_ORIGIN_ROBOT`) and the
URScript `set_tcp()` value are unchanged between markers.

**How it cycles** (in `ur3_drawing_node.py`):

```
For each stroke s_idx:
    marker_idx = s_idx % 4              # 0, 1, 2, 3, 0, 1, 2, 3, …
    quat = TOOL_QUAT ⊗ Rz(-90° × marker_idx)
    plan travel + draw + lift waypoints with `quat` as orientation
```

MoveIt2 then naturally rotates wrist_3 by 90° during the pen-up travel
between strokes (when the EE is safely above the canvas), swapping the
active marker without any extra explicit rotation step.

| Stroke index | Marker | wrist_3 offset (rel. to baseline) |
|--------------|--------|------------------------------------|
| 0, 4, 8, …   | 1      | 0°                                 |
| 1, 5, 9, …   | 2      | -90°                               |
| 2, 6, 10, …  | 3      | -180°                              |
| 3, 7, 11, …  | 4      | -270° (= +90°)                     |

If your physical holder loads markers in a different order, swap the
colour cartridges in the holder rather than changing the code — the
mapping above is the deterministic order of use.

### Drawing at Angle: Combining 20° Tilt + Cartesian Paths

**Goal:** Draw on a canvas while the marker is tilted 20° from vertical.

**Challenge:** Most robot drawing just points the tool straight down. Our system points it at an angle.

**Solution:** The system works in two stages:

1. **MoveIt2 planning stage** (online):
   - Waypoints are commanded as **Cartesian poses** (x, y, z, quaternion)
   - The quaternion encodes the tool orientation: tilted 20° around the Y-axis
   - MoveIt2 plans collision-free joint-space interpolations between these poses
   - The marker stays at 20° throughout the trajectory

2. **URScript execution stage**:
   - The planned **joint positions** are converted to `movej()` commands
   - `set_tcp()` is called once with the offset
   - The robot executes `movej()` to each joint configuration → marker follows the planned Cartesian path at the desired angle

**Example:** Drawing a straight line on the canvas

1. **Perception** sends 100 stroke points (pixels)
2. **Motion node** converts pixels → robot XYZ (e.g., `[0.240, -0.075, 0.11]` = first draw point)
3. **Pose for MoveIt2** is constructed: position + quaternion for 20° tilt
   ```python
   pose.position = (0.240, -0.075, 0.11)  # xyz
   pose.orientation = (0.9848, 0.0, -0.1736, 0.0)  # 20° tilt around Y
   ```
4. **MoveIt2** plans a Cartesian line → `[q1, q2, q3, q4, q5, q6]` (100 joint configs)
5. **URScript** is built with 100 `movej()` commands
6. **Robot** executes → marker draws a line at 20°, reaching down to the canvas

### Why This Approach?

- **Collision safety:** MoveIt2 knows about the table and marker holder shape, so it never plans paths that crash
- **Correct tool orientation:** The quaternion in the pose ensures the 20° angle is maintained throughout
- **Efficient execution:** URScript is lightweight and works on both simulator and real robot without UR driver infrastructure
- **Hybrid:** Leverages MoveIt2's planning strengths while avoiding heavy action server dependencies

---

## How to Run with MoveIt2

### Launch MoveIt2 + Draw (with Collision Avoidance)

**Simulator:**
```bash
# Build both packages (required after any code changes)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Terminal 1 — Full integrated pipeline with MoveIt2 + Perception + Motion
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  launch_rviz:=true

# Terminal 2 — Start the GUI
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py

# Terminal 3 (optional) — Start the UR3 simulator
export ROS_DOMAIN_ID=42
ros2 run ur_client_library start_ursim.sh -m ur3
```

**Real Robot:**
```bash
# Build both packages (required after any code changes)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Terminal 1
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  robot_ip:=192.168.0.195 \
  launch_rviz:=true

# Terminal 2 — GUI (same as simulator)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py
```

### Launch MoveIt2 Only (for scene setup or debugging)

```bash
# Build the motion package
source ~/RS2/ros2_ws/install/setup.bash
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Launch
export ROS_DOMAIN_ID=42
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101 \
  launch_rviz:=true
```

This starts:
- `move_group` (MoveIt2 planning server)
- `RViz` (3D visualization)
- Scene setup node that publishes table + marker holder

You can then:
- Use RViz to visualize the table and marker holder
- Test trajectories with the MoveIt2 GUI in RViz
- Verify collision checking works

---

## QA Checklist

- [x] 3-way ROS 2 integration (GUI ↔ Perception ↔ Motion)
- [x] 20° marker tilt with quaternion representation
- [x] 15 cm EE height above canvas
- [x] MoveIt2 `/compute_cartesian_path` planning
- [x] Collision objects (table, marker holder attached to tool0)
- [x] URScript generation from planned trajectories
- [x] Real robot support (configurable IP)
- [x] Perception-published strokes flow to motion node
- [x] GUI status feedback from motion node
- [x] **Four-marker rotation (wrist_3 swaps colour per stroke)**
- [ ] Full end-to-end test on real robot with all 4 colours (pending team scheduling)
