# UR3 Selfie Drawing Robot - Motion Planning Pipeline

**Team Picasso | Motion Planning Subsystem | Domenic Kadioglu**

---

## ⚡ NEW: ROS 2 Integration (Sprint 2)

This project now includes a **ROS 2 Humble node** for system-level integration with Perception and GUI subsystems.

### Architecture Overview
```
Perception (Nithish)        Motion Planning (YOU)         GUI (Mateusz)
     ↓                            ↓                            ↓
/stroke_paths topic → motion_planning_node → /urscript_program
                            ↓
                      ur_robot_driver / Polyscope
                            ↓
                        UR3 Robot
```

### Quick Start
```bash
# Build and test ROS 2 node
cd ~/RS2/ros2_ws
colcon build
source install/setup.bash
ros2 launch ur3_motion_planning ur3_motion_planning.launch.py robot_ip:=192.168.56.101
```

**See [TESTING_AND_UR3_GUIDE.md](./TESTING_AND_UR3_GUIDE.md) for complete testing instructions.**

### Key Features
- ✓ Subscribes to `/stroke_paths` from Perception team
- ✓ Generates optimized trajectories (NN + 2-Opt)
- ✓ Publishes `/urscript_program` for execution
- ✓ Publishes `/planning_status` for GUI feedback
- ✓ Works with Polyscope simulator and real UR3
- ✓ Original ur3_selfie_draw.py code unchanged (backward compatible)

**Full documentation:** [ros2_ws/src/ur3_motion_planning/README.md](./ros2_ws/src/ur3_motion_planning/README.md)

---

## Directory Structure

```
/home/domenic/RS2/
├── README.md                                # This file
├── TESTING_AND_UR3_GUIDE.md                 # Quick testing guide
├── SPRINT2_TEST_PLAN.md                     # Test specifications
├── SPRINT2_TEST_EVIDENCE.md                 # Test results
├── SPRINT2_PRESENTATION.md                  # Presentation slides

├── src/                                     # Original Python scripts
│   ├── svg_to_json_converter.py             # SVG → JSON conversion
│   └── ur3_selfie_draw.py                   # Motion planning (615 lines)

├── ros2_ws/                                 # ROS 2 Workspace (NEW)
│   ├── build/                               # Build outputs
│   ├── install/                             # Installed packages
│   └── src/
│       └── ur3_motion_planning/             # Motion planning ROS 2 node
│           ├── ur3_motion_planning/
│           │   ├── motion_planning_node.py  # Main ROS 2 node
│           │   └── __init__.py
│           ├── launch/
│           │   └── ur3_motion_planning.launch.py
│           ├── package.xml
│           ├── setup.py
│           └── README.md                    # Full ROS 2 documentation

├── inputs/                                  # Your SVG files
│   └── face1.svg

├── outputs/                                 # Generated results
│   ├── strokes/                             # JSON stroke data
│   │   └── face1_strokes.json
│   └── verified/                            # Verification SVGs
│       └── face1_verified.svg

└── __pycache__/
```

## Overview

Complete workflow for converting hand-drawn faces into UR3 robot drawing trajectories.

**Process:**
1. Draw on online editor → Export SVG to `inputs/`
2. SVG → JSON stroke converter (with verification)
3. Motion planning optimization
4. Performance timing analysis

---

## Step-by-Step Workflow

### Step 1: Draw Your Image

Use **[method.ac](https://editor.method.ac/)** online SVG editor:
- Simple interface: click and draw
- Lines automatically become SVG paths
- **Download/Export as SVG file**

**Save location:**
```
inputs/your_drawing.svg
```

**Tips for good results:**
- Keep lines continuous (fewer strokes = faster optimization)
- Simple shapes work best (faces, geometric patterns)
- Don't worry about size — the script automatically scales drawings to fit the UR3 workspace

---

### Step 2: Convert SVG to JSON

From the project root directory (`/home/domenic/RS2/`):

```bash
python3 src/svg_to_json_converter.py
```

**What it does:**
- Reads SVG from `inputs/` folder (edit script for filename)
- Extracts all `<path>` elements
- Parses SVG commands: `M` (move), `L` (line), `C` (bezier curves)
- Approximates bezier curves with 10-segment line segments
- Outputs JSON with stroke coordinates

**Output files:**
- `outputs/strokes/your_drawing_strokes.json` — Stroke coordinates
- `outputs/verified/your_drawing_verified.svg` — SVG reconstructed from JSON

**Verification:**
Open both SVGs in a browser side-by-side:
- `inputs/your_drawing.svg` (original)
- `outputs/verified/your_drawing_verified.svg` (from JSON)

If they look identical, conversion is correct ✓

---

### Step 3: Run Motion Planning

From the project root directory:

```bash
python3 src/ur3_selfie_draw.py
```

**Configuration:**
Edit this line in `src/ur3_selfie_draw.py`:
```python
json_file = 'outputs/strokes/your_drawing_strokes.json'  # Change to your file
```

**Console output:**
```
============================================================
UR3 MOTION PLANNING PIPELINE
============================================================

✓ Loaded 13 strokes from 'outputs/strokes/face1_strokes.json'
  Total waypoints: 1294
============================================================

[Processing logs...]

============================================================
EXECUTION SUMMARY
============================================================

  Strokes: 13 | Waypoints: 1294
  Travel distance: 1361.1 px (saved 30.8%)
  NN time: 1.1ms | 2-Opt time: 5.4ms | Script time: 14.5ms
        
Total execution time: 0.022s
============================================================
```

---

## What Happens at Each Stage

### Stage 1: Stroke Loading
- Reads JSON file with pixel coordinates `(x, y)`
- Validates format: `List[List[Tuple[float, float]]]`
- Counts total waypoints

### Stage 2: Automatic Workspace Scaling
- **Goal:** Fit drawing within UR3 reachable workspace
- Detects bounding box of all strokes
- Scales uniformly to fit within 95% of canvas bounds
- Centers drawing on canvas origin
- **Benefit:** No more inverse kinematics (IK) errors from unreachable poses
- **Example:** 217×393px drawing scaled to 72.4% to fit within 400×300px canvas target

### Stage 3: Nearest-Neighbour Optimization (NN Sort)
- **Goal:** Minimize pen-up travel distance (idle time)
- **Algorithm:** Greedy nearest-neighbour heuristic
- **Complexity:** O(n²) on number of strokes
- **Output:** Reordered strokes, travel distance savings

Example: 13 strokes, 902 px saved (45% improvement)

### Stage 4: 2-Opt Local Search Refinement
- **Goal:** Further optimize stroke ordering
- **Algorithm:** Swap adjacent strokes if it reduces travel
- **Iterations:** Up to 50 passes (stops when no improvement)
- **Output:** Fine-tuned stroke order

Example: 4 iterations, -295.6 px additional savings

### Stage 5: Pixel → Robot Coordinate Transform
- Converts pixel coordinates to UR3 world frame
- **Canvas origin:** Calibrated robot position `[0.350, -0.150, 0.010]` m
- **Canvas size:** 0.200 m × 0.150 m (20 cm × 15 cm)
- **Z heights:**
  - Pen-down (drawing): 0.010 m
  - Pen-up (travel): 0.060 m

### Stage 6: URScript Generation
- Generates URScript program with `movel()` commands
- Includes:
  - Movement to safe home position
  - Pen-up/pen-down transitions
  - Workspace collision validation
  - Tool orientation (pen pointing down)
- **Output:** Full robot program (in memory, not saved to file)

---

## Performance Metrics

### Execution Time Breakdown

| Component | Time | Notes |
|-----------|------|-------|
| JSON loading | <1 ms | File I/O |
| Stroke scaling | <1 ms | Bounding box detection & scaling |
| NN optimization | 1-5 ms | Depends on stroke count |
| 2-Opt refinement | 5-80 ms | Up to 50 iterations |
| Script generation | 8-30 ms | URScript building |
| **Total** | **20-120 ms** | Not including robot execution |

### Drawing Time (on actual robot)

**Not** included in execution time above. Depends on:
- Number of waypoints (1,294 in face example)
- Linear velocity setting (default 0.05 m/s = slow/precise)
- Distance traveled

**Estimate:** 3-5 minutes for typical face drawing

---

## Testing Multiple Drawings

### Create More Test Files

1. Draw new image on [method.ac](https://editor.method.ac/)
2. Export as SVG → save to `inputs/`
3. Edit `src/svg_to_json_converter.py`:
   ```python
   svg_name = 'face2'  # Change this
   ```
4. Convert:
   ```bash
   python3 src/svg_to_json_converter.py
   ```
5. Edit `src/ur3_selfie_draw.py`:
   ```python
   json_file = 'outputs/strokes/face2_strokes.json'
   ```
6. Test:
   ```bash
   python3 src/ur3_selfie_draw.py
   ```

### Batch Testing

Compare performance across multiple drawings:

| Drawing | Strokes | Waypoints | NN Savings | 2-Opt Savings | Total Time | Est. Robot Time |
|---------|---------|-----------|-----------|---------------|------------|-----------------|
| Face 1 | 13 | 1,294 | 45% | -15% | 0.022s | ~4 min |
| (Add more...) | | | | | | |

---

## File Locations Reference

| Purpose | Location | Created by |
|---------|----------|------------|
| Original drawing | `inputs/face1.svg` | You (method.ac export) |
| JSON strokes | `outputs/strokes/face1_strokes.json` | svg_to_json_converter.py |
| Verification SVG | `outputs/verified/face1_verified.svg` | svg_to_json_converter.py |
| Motion plan script | `src/ur3_selfie_draw.py` | (stays in place) |
| SVG converter script | `src/svg_to_json_converter.py` | (stays in place) |

---

## Polyscope Testing

### Start the UR3 Simulator

Run these two commands in your terminal (they must run sequentially):

```bash
docker pull universalrobots/ursim_cb3
ros2 run ur_client_library start_ursim.sh -m ur3
```

What happens:
1. First command downloads the UR simulator Docker image
2. Second command starts the simulator

The simulator will be running in the background.

### Run Motion Plan on Polyscope

In a **new terminal** (while Polyscope simulator is running), run the motion planning script:

```bash
cd /home/domenic/RS2
python3 src/ur3_selfie_draw.py
```

The script automatically:
- Connects to Polyscope
- Generates the optimized drawing trajectory
- Sends the program to the robot simulator
- Prints status and timing metrics

**Console output:**
```
✓ Loaded 13 strokes from 'outputs/strokes/face1_strokes.json'
  Total waypoints: 1294

[Debug Info]
  Home position: [0.3, -0.225, 0.1]
  Canvas origin: [0.35, -0.15, 0.01]
  Canvas bounds X: [0.300, 0.600] m
  Canvas bounds Y: [-0.350, -0.100] m
...
[Scaling] Stroke bounds: X [279, 496], Y [107, 500]
[Scaling] Stroke size: 217×393px, scale_factor: 0.724
[Scaling] Rescaled to 72.4% to fit within canvas
...

Strokes: 13 | Waypoints: 1294
Travel distance: 739.8 px (saved 41.0%)
NN time: 1.1ms | 2-Opt time: 79.0ms | Script time: 8.0ms

Total execution time: 0.090s

[Robot] Connected to 192.168.56.101:30002
[Sending drawing program to robot...]
[Ready to run! Click 'Play' in Polyscope to execute drawing.
```

Watch the Polyscope simulator window to see the robot arm performing the drawing with pen-up/pen-down transitions.

### IK Error Prevention & Motion Strategy

**Automatic stroke scaling prevents inverse kinematics errors:**

The code now automatically detects if your drawing exceeds UR3 workspace bounds and scales it down proportionally:
- **Detects bounding box** of all strokes
- **Calculates scale factor** to fit within 95% of canvas
- **Scales uniformly** to preserve drawing aspect ratio
- **No manual calibration needed** — works with any SVG size
- **Example:** A 217×393px drawing automatically scaled to 72.4% to fit within reachable workspace

**Motion strategy for smooth, safe execution:**

- **Home position:** `movel()` (linear motion) — safe approach to starting position
- **Pen-up travel:** `movel()` with high velocity — smooth, fast transitions
- **Pen-down drawing:** `movel()` with controlled velocity — ensures precise lines
- **Speed tuning:** Optimized acceleration/velocity for simulator stability
- **Workspace validation:** 30cm margin accommodates calibration variations and SVG scaling

**What the robot does:**
1. Move to home position above canvas (safe, smooth)
2. Move to pen-up position above first stroke
3. Lower pen and start drawing
4. Move through stroke waypoints (smooth line)
5. Lift pen (pen-up state)
6. Move to next stroke
7. Repeat for each stroke

This approach balances speed with accuracy and eliminates unreachable pose errors.

---

## Next Steps

### For Real Robot Testing
1. Calibrate canvas position on physical UR3 arm
2. Update `CANVAS_ORIGIN_ROBOT` in script with actual calibration
3. Verify workspace bounds don't exceed robot limits
4. Run on actual UR3 with the generated script

### For Polyscope Integration
- Script already generates URScript (use output capture method above)
- Full bidirectional integration possible via Polyscope REST API
- Monitor execution feedback from robot simulator

### For Optimization
- Adjust `CANVAS_PX_W`, `CANVAS_PX_H` for different image sizes
- Tune `LINEAR_VEL` for drawing quality vs. speed
- Implement trajectory smoothing for time optimization

---

## Configuration Parameters

In `src/ur3_selfie_draw.py`:

```python
# Canvas size (pixels) - target SVG dimensions
CANVAS_PX_W = 400
CANVAS_PX_H = 300

# Robot workspace (metres)
CANVAS_ORIGIN_ROBOT = np.array([0.350, -0.150, 0.010])
CANVAS_WIDTH_M = 0.200     # 20 cm
CANVAS_HEIGHT_M = 0.150    # 15 cm

# Safe home position (definitely reachable)
HOME_POS = np.array([0.300, -0.225, 0.100])

# Heights (metres)
Z_DRAW = 0.010     # Pen down
Z_TRAVEL = 0.060   # Pen up

# Motion speeds
JOINT_ACCEL = 1.4  # rad/s² (simulator speeds)
JOINT_VEL = 1.05   # rad/s
LINEAR_ACCEL = 0.5 # m/s²
LINEAR_VEL = 0.08  # m/s (drawing speed)

# Optimization
2-OPT_MAX_ITERATIONS = 50  # Refinement passes (reduce for speed)
```

### Automatic Scaling Behavior

The script automatically scales strokes to fit your UR3 workspace:

1. **Detects actual stroke bounds** from your SVG/JSON
2. **Calculates scale factor** = available_canvas / actual_stroke_size
3. **Scales proportionally** (aspect ratio preserved)
4. **95% canvas fill** — leaves 5% margin for safety
5. **No manual intervention needed** — works with any sized drawing

You can draw large images on method.ac and the script handles fitting them to the robot.

### Calibration Tips

- **Canvas origin** — Update `CANVAS_ORIGIN_ROBOT` to your actual robot's canvas position
- **Home position** — Adjust `HOME_POS` if your workspace layout differs
- **Canvas bounds** — Update `CANVAS_WIDTH_M` and `CANVAS_HEIGHT_M` to match your physical setup
- **Speed tuning** — Increase `LINEAR_VEL` for faster execution, decrease for more precision

---

## Troubleshooting

### JSON file not found
```
ERROR: JSON file not found
File to load: outputs/strokes/face1_strokes.json
```
→ Check that file exists in `outputs/strokes/`
→ Run `python3 src/svg_to_json_converter.py` first

### SVG conversion looks wrong
→ Compare `inputs/face1.svg` vs `outputs/verified/face1_verified.svg`
→ Check curve approximation (edit converter for more segments)

### Inverse kinematics (IK) errors in Polyscope
→ **These are now handled automatically** — the script scales drawings to fit workspace
→ If errors persist, check that scaling is applied (look for `[Scaling]` log messages)
→ Verify `CANVAS_ORIGIN_ROBOT` position is correctly calibrated

### Drawing runs slowly in Polyscope
→ Check if 2-Opt optimization is taking too long (reduce `2-OPT_MAX_ITERATIONS` to 20)
→ Increase `LINEAR_VEL` for faster drawing (but reduces precision)

### Robot doesn't move at all
→ Verify Polyscope simulator is running and reachable at `192.168.56.101:30002`
→ Check that home position `[0.3, -0.225, 0.1]` is within robot reach
→ Review console output for workspace warnings

---

## Summary

**Workflow:**
1. **Draw** online at any size → SVG file to `inputs/`
2. **Convert** SVG → JSON strokes (verified with SVG output)
3. **Optimize** JSON → Automatically scales + optimizes trajectory
4. **Test** on Polyscope or real UR3

**Ready for:**
- Polyscope simulator testing
- Real UR3 robot execution
- Performance benchmarking across multiple drawings

**Key advantages:**
- ✅ **Automatic workspace scaling** — handles any SVG size
- ✅ Clean directory organization
- ✅ Fast conversion & optimization (20-120 ms including scaling)
- ✅ Visual verification of conversions
- ✅ Detailed timing and scaling metrics
- ✅ No IK errors — unreachable poses handled automatically
- ✅ Easy to compare multiple drawings
