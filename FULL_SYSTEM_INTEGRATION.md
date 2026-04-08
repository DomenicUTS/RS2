# Full System Integration — GUI + Perception + Motion Planning

**Project:** UR3 Selfie Drawing Robot — Team Picasso  
**Date:** 9 April 2026  
**Contributors:** GUI student, Perception student, Domenic Kadioglu (Motion)

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
| `/raw_image` | `sensor_msgs/Image` | **GUI** node or **image_loader_node** | face_detection_node, mapping_node (reset), visualization_node (reset) | Raw camera frame (BGR8) |
| `/aligned_face` | `sensor_msgs/Image` | face_detection_node | image_processing_node | Cropped 256×256 face |
| `/processed_mask` | `sensor_msgs/Image` | image_processing_node | mapping_node | Edge-detected contour mask |
| `/drawing_strokes` | `std_msgs/String` | mapping_node | **ur3_drawing_node**, **GUI** node, visualization_node | JSON stroke array `[[[x,y],…],…]` in 400×300 px |
| `/drawing_preview_image` | `sensor_msgs/Image` | visualization_node | **GUI** node | Rendered preview of what robot will draw |
| `/drawing_status` | `std_msgs/String` | **ur3_drawing_node** | mapping_node, **GUI** node | Pipeline state (WAITING, EXECUTING, COMPLETE, ERROR, …) |
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
| **Subscribes** | `/drawing_status` | Logs motion pipeline state |
| **Publishes** | `/drawing_strokes` | JSON strokes after edge detection + mapping |
| **Publishes** | `/drawing_preview_image` | Rendered preview of strokes |
| **File output** | `~/perception/output/perception_strokes.json` | Same strokes saved to disk |

**Nodes:**
- `image_loader_node` — Watches `~/perception/input/` for images (standalone testing)
- `camera_node` — Live webcam at 10 FPS (alternative to image_loader)
- `face_detection_node` — Haar cascade face crop → 256×256
- `image_processing_node` — Elliptical mask + Canny edge detection
- `mapping_node` — Contour extraction, Douglas-Peucker simplification, stroke ordering
- `visualization_node` — Stroke preview rendering + file save

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

### Mode 1: Full Integration (GUI + Perception + Motion)

```bash
# Terminal 1 — Start the backend pipeline
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py image_source:=gui

# Terminal 2 — Start the GUI
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py

# Terminal 3 (optional) — UR3 simulator
ros2 run ur_client_library start_ursim.sh -m ur3
```

**User workflow:** Camera preview → Capture → Process (sends to perception) → Preview appears → Start Drawing → Robot executes.

### Mode 2: Perception + Motion (no GUI)

```bash
# Place an image in ~/perception/input/
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py
```

The `image_loader_node` will automatically find and publish the image.

### Mode 3: Each Subsystem Standalone

**GUI only** (fake processing, no ROS needed):
```bash
cd ~/gui
python3 selfie_drawing_gui_starter.py
```

**Perception only** (process an image, no robot):
```bash
source ~/perception/install/setup.bash
ros2 launch selfie_perception perception_pipeline.launch.py
# OR standalone test (no ROS at all):
python3 ~/perception/src/selfie_perception/selfie_perception/test_perception.py
```

**Motion only** (draw from pre-made JSON, no perception):
```bash
# Direct Python (no ROS):
cd ~/RS2 && python3 src/ur3_selfie_draw.py 1

# Via ROS node (file mode):
source ~/RS2/ros2_ws/install/setup.bash
ros2 run ur3_motion_planning motion_planning_node --ros-args -p face:=face1
```

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
| `image_loader_node.py` | Fixed default `image_dir` to `~/perception/input/` (was broken relative path) |
| `mapping_node.py` | Fixed default `output_dir`. Added `/drawing_status` subscriber (feedback). Added `/raw_image` subscriber to reset `processed` flag for multi-capture. |
| `visualization_node.py` | Fixed default `output_dir`. Added `drawing_preview_image` publisher (Image). Added `/raw_image` subscriber to reset `done` flag. Added `cv_bridge` import. |
| `camera_node.py` | Unchanged |
| `face_detection_node.py` | Unchanged |
| `image_processing_node.py` | Unchanged |

### Motion Planning (`~/RS2/`)

| File | Change |
|------|--------|
| `ur3_drawing_node.py` | Added `/gui/command` subscriber (START/PAUSE/RESUME/STOP). Updated `_on_drawing_strokes` to store new strokes even after first run. |
| `integrated_pipeline.launch.py` | Added `image_source` argument (file/gui). Conditionally launches `image_loader_node`. |
| `ur3_selfie_draw.py` | No changes in this round (20° tilt + 10cm height from previous integration). |

---

## Architecture Principles

1. **Loose coupling via topics** — each module only knows about topic names and message types, not about each other's internals.
2. **Standalone first** — every module can run and be tested independently. Integration is additive.
3. **Single source of truth** — stroke format is defined once (400×300 px JSON). No translation layers needed.
4. **Reset-capable** — perception and motion nodes can process multiple images in a session (the GUI can retake and re-process).
