# Perception ↔ Motion Planning Integration Guide

**Project:** UR3 Selfie Drawing Robot — Team Picasso  
**Date:** 8 April 2026  
**Author:** Domenic Kadioglu (Motion Planning) + Perception Subsystem Student

---

## Overview

This document describes how the **selfie_perception** pipeline (face detection,
edge extraction, stroke mapping) is connected to the **ur3_motion_planning**
pipeline (path optimisation, URScript generation, robot execution) using **ROS 2
topics**.

Before this integration the two subsystems operated independently: perception
saved a JSON file and motion planning loaded it from disk.  Now they can also
communicate in real-time over a shared ROS 2 topic, removing the manual
file-copy step.

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  selfie_perception                      │
│                                                         │
│  image_loader_node ──→ face_detection_node              │
│         (raw_image)       │                             │
│                           ▼                             │
│                   image_processing_node                 │
│                      (aligned_face)                     │
│                           │                             │
│                           ▼                             │
│                      mapping_node ──────────────────────┼──→ /drawing_strokes
│                           │             (std_msgs/String │     (JSON)
│                           ▼              with JSON)      │
│                   visualization_node                    │
│                  (drawing_preview.png)                   │
│                                                         │
│  ◄──────── /drawing_status (feedback from motion) ──────┼──
└─────────────────────────────────────────────────────────┘

                            │
                            │  /drawing_strokes
                            ▼

┌─────────────────────────────────────────────────────────┐
│                  ur3_motion_planning                     │
│                                                         │
│  ur3_drawing_node                                       │
│    • subscribes to /drawing_strokes (topic mode)        │
│    • OR loads from JSON file (file mode)                │
│    • optimises path (NN + 2-Opt)                        │
│    • generates URScript with set_tcp() for marker       │
│    • sends to robot or saves script                     │
│                                                         │
│  ──→ /drawing_status   (pipeline state updates)         │
│  ──→ /trajectory_preview (PoseArray for RViz)           │
└─────────────────────────────────────────────────────────┘
```

---

## ROS 2 Topics (Integration Points)

| Topic | Type | Publisher | Subscriber | Description |
|---|---|---|---|---|
| `/drawing_strokes` | `std_msgs/String` | `mapping_node` (perception) | `ur3_drawing_node` (motion) | JSON array of strokes: `[[[x,y],…],…]` in 400×300 px space |
| `/drawing_status` | `std_msgs/String` | `ur3_drawing_node` (motion) | `mapping_node` (perception) | Pipeline state: `WAITING_FOR_PERCEPTION`, `LOADING_STROKES`, `OPTIMIZING_PATH`, `EXECUTING`, `COMPLETE`, etc. |

---

## How to Run

### Option A — Integrated launch (recommended)

A single launch file starts **both** subsystems.  The motion node starts in
`topic` mode and waits for strokes from perception.

```bash
# Terminal 1 — (optional) Start UR3 simulator
ros2 run ur_client_library start_ursim.sh -m ur3

# Terminal 2 — Build both workspaces
cd ~/perception && colcon build --symlink-install
cd ~/RS2/ros2_ws && colcon build --symlink-install

# Source BOTH workspaces (order matters — source perception first)
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash

# Terminal 3 — Launch integrated pipeline
ros2 launch ur3_motion_planning integrated_pipeline.launch.py
```

Place an input image in `~/perception/input/` before launching.  The
perception pipeline will detect the face, extract edges, and publish strokes.
The motion node will automatically pick them up, optimise the path, and send
the URScript to the robot.

### Option B — Separate terminals (more control)

```bash
# Terminal 1 — Perception pipeline
source ~/perception/install/setup.bash
ros2 launch selfie_perception perception_pipeline.launch.py

# Terminal 2 — Motion planning (topic mode)
source ~/perception/install/setup.bash   # need perception msgs on path
source ~/RS2/ros2_ws/install/setup.bash
ros2 run ur3_motion_planning motion_planning_node \
    --ros-args -p stroke_source:=topic

# Terminal 3 — Monitor
ros2 topic echo /drawing_status
ros2 topic echo /drawing_strokes
```

### Option C — File-based (original workflow, still works)

```bash
# 1. Run perception standalone
cd ~/perception
python3 src/selfie_perception/selfie_perception/test_perception.py

# 2. Copy output
cp ~/perception/output/perception_strokes.json \
   ~/RS2/outputs/strokes/face1_strokes.json

# 3. Run motion planning
cd ~/RS2
python3 src/ur3_selfie_draw.py 1
```

---

## Changes Made

### ur3_motion_planning (Motion — Domenic's code)

#### `src/ur3_selfie_draw.py`

| Change | Detail |
|--------|--------|
| **Marker holder constants** | Added `MARKER_TILT_DEG = 20.0`, `EE_DRAW_HEIGHT = 0.10` m |
| **Z_DRAW** | Changed from `0.010` (at canvas) to `CANVAS_SURFACE_Z + 0.10` = `0.110` m (end effector 10 cm above canvas) |
| **Z_TRAVEL** | Changed from `+0.150` above Z_DRAW to `+0.060` above new Z_DRAW = `0.170` m |
| **TOOL_ORIENT** | Computed via rotation-matrix composition: Rx(180°) then Ry(20°). No longer `[π, 0, 0]` |
| **TCP_OFFSET** | New constant — horizontal and vertical offset of marker tip from flange |
| **set_tcp()** | Added to generated URScript so the robot knows where the marker tip is |
| **Rotation helpers** | `_rotation_matrix_to_rotvec()` and `compute_tilted_tool_orient()` |

#### `ur3_motion_planning/ur3_drawing_node.py`

| Change | Detail |
|--------|--------|
| **`stroke_source` parameter** | `'file'` (default, original) or `'topic'` (subscribe to perception) |
| **`/drawing_strokes` subscriber** | Listens for `std_msgs/String` with JSON strokes |
| **`_on_drawing_strokes()` callback** | Parses JSON, stores strokes, auto-starts pipeline in topic mode |
| **`_load_strokes()` updated** | Checks topic data first when `stroke_source='topic'` |
| **New imports** | `MARKER_TILT_DEG`, `EE_DRAW_HEIGHT`, `TCP_OFFSET` |

#### `package.xml`

- Added `<exec_depend>selfie_perception</exec_depend>`.

#### `setup.py`

- Added `integrated_pipeline.launch.py` to installed launch files.

#### New file: `launch/integrated_pipeline.launch.py`

- Launches all perception nodes + motion node in topic mode.

### selfie_perception (Perception — other student's code)

#### `selfie_perception/mapping_node.py`

| Change | Detail |
|--------|--------|
| **`/drawing_status` subscriber** | Listens for status feedback from motion pipeline |
| **`_on_drawing_status()` callback** | Logs status messages (informational) |

No other perception files were modified.

---

## Marker Holder Geometry (20° Tilt at 10 cm)

The 3D-printed marker holder attaches to the UR3 end effector and holds a
marker at **20° from perpendicular**.  The end effector maintains a height of
**10 cm** above the canvas surface during drawing.

```
         ┌──── End Effector (flange)
         │
         │  10 cm
         │  ↕
         │╲
         │ ╲  20°
         │  ╲
         │   ╲  ← marker
         │    ╲
    ─────┴─────╳──── Canvas surface
              marker tip
```

### Computed values

| Parameter | Value | Formula |
|-----------|-------|---------|
| Horizontal TCP offset | ~3.42 cm | `10 × sin(20°)` |
| Vertical TCP offset | ~−9.40 cm | `−10 × cos(20°)` |
| End effector Z (draw) | 0.110 m | `canvas_z + 0.10` |
| End effector Z (travel) | 0.170 m | `Z_DRAW + 0.06` |
| Tool orientation | `[3.09, 0.0, −0.55]` (approx) | `Ry(20°) × Rx(180°)` → rotation vector |

The `set_tcp()` URScript command is emitted at the top of every generated
program so the robot controller accounts for the marker offset in all
subsequent `movel()` commands.

---

## Data Format Compatibility

Both subsystems use the same JSON stroke format:

```json
[
  [[x1, y1], [x2, y2], ...],
  [[x1, y1], [x2, y2], ...],
  ...
]
```

- **Coordinate space:** 400 × 300 pixels (configurable via `canvas_px_w`/`canvas_px_h`)
- **Origin:** top-left, Y increases downward
- **Units:** floating-point pixel values (1 decimal place from perception)

The motion pipeline's `scale_strokes_to_workspace()` will rescale if the
incoming strokes exceed the canvas pixel bounds.
