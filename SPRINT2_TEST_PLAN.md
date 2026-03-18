# Sprint 2 - Motion Planning and Control Subsystem Test Plan
**Team Picasso | Domenic Kadioglu**  
**Due:** 2 April 2026

---

## Subsystem Overview

**Lead:** Domenic Kadioglu  
**Subsystem:** Motion Planning and Control (UR3 Trajectory Generation)  

**Role:** Converts vector paths from perception subsystem into safe, optimized trajectories for UR3 robot execution. Generates URScript programs, validates poses against workspace constraints, and communicates with Polyscope simulator.

**Key Requirements (from Project Contract - Pass Level):**
- Generates trajectories from vector paths
- Implements pen-up/pen-down motion control
- Ensures safe motion (no joint limit violations, workspace validation)
- Executes drawing in < 2 minutes 45 seconds
- Executes trajectories without failure

---

## Test 1: Trajectory Generation and Execution Accuracy

**Requirement:** System successfully generates valid URScript trajectories from stroke vectors without errors, and executes them on Polyscope simulator without IK failures.

**Assumptions:**
- Polyscope simulator is running (Docker: `ros2 run ur_client_library start_ursim.sh -m ur3`)
- Input strokes are in correct JSON format (13 test strokes from face1.json)
- Robot home position is reachable [0.300, -0.225, 0.100]
- TCP connection to port 30002 is available

**Procedure:**
1. Place valid stroke JSON file at `inputs/face1.json`
2. Run pipeline: `python3 src/ur3_selfie_draw.py`
3. Capture console output logging:
   - Scaling detection output
   - Stroke loading confirmation
   - NN sort and 2-Opt optimization metrics
   - Script generation success
   - Polyscope connection status
   - Script transmission status message
4. Verify output in Polyscope:
   - Check that script was received (should see "draw_face()" program)
   - Run script and watch robot execution
   - Note any E-stop triggers or IK errors

**Pass/Fail Criteria:**
- ✓ PASS: 
  - Script generates without errors
  - Console shows "[Robot] Connected to 192.168.56.101:30002"
  - Script sent successfully (byte count logged)
  - Robot executes strokes without E-stop or IK failures
  - All 13 strokes complete pen-up/pen-down transitions
- ✗ FAIL:
  - Python runtime errors
  - Connection failed to simulator
  - IK solution not found errors in Polyscope
  - Script execution interrupted by E-stop

**Evidence:**
- Test runs: 3 consecutive full executions
- Log output: Console output showing metrics
- Video: Screen recording of Polyscope execution (optional)

---

## Test 2: Workspace Validation and Safe Motion

**Requirement:** Trajectory generation validates all poses against workspace bounds and prevents sending unreachable positions to robot.

**Assumptions:**
- Canvas origin calibrated at [0.350, -0.150, 0.010] m
- Safe workspace margins set to ±0.30 m
- Automatic scaling feature is functional
- Home position [0.300, -0.225, 0.100] is guaranteed reachable

**Procedure:**
1. Load test file with deliberately oversized strokes (e.g., face1.json with 500px canvas)
2. Run `python3 src/ur3_selfie_draw.py`
3. Capture scaling output logs:
   - Detected stroke bounds (X min/max, Y min/max in pixels)
   - Calculated scale factor
   - "Rescaled to X% to fit within canvas" message
4. Verify pose validation:
   - Check console for any workspace violation warnings
   - Count total validation warnings (should be 0 after scaling)
5. Examine generated URScript:
   - All movel() commands should have coordinates within bounds
   - Z heights: 0.010 m (pen-down) and 0.060 m (pen-up)

**Pass/Fail Criteria:**
- ✓ PASS:
  - Scaling automatically detects oversized strokes
  - Scale factor applied (e.g., 0.724 = 72.4%)
  - Final script contains 0 workspace violation warnings
  - All coordinates in URScript are within [-0.35 to -0.10] m (Y axis)
  - Robot accepts and executes all posed without IK errors
- ✗ FAIL:
  - Scaling does not activate for oversized input
  - Workspace violations remain after scaling
  - Script rejected by Polyscope due to unreachable poses
  - Manual re-scaling required

**Evidence:**
- Console output logs showing scaling detection
- Before/after comparison: stroke bounds vs canvas bounds
- URScript excerpt showing coordinate ranges

---

## Test 3: Path Optimization and Execution Time

**Requirement:** Motion planning achieves minimum 20-30% reduction in non-drawing travel distance and completes execution in < 2 minutes 30 seconds.

**Assumptions:**
- NN (Nearest-Neighbour) optimization is enabled
- 2-Opt refinement runs with 50 max iterations
- Input is 13 strokes from face1.json (1,294 waypoints)
- Execution time measured from script start to home return

**Procedure:**
1. Run: `python3 src/ur3_selfie_draw.py`
2. Capture timing metrics from console output:
   - Raw travel distance (before optimization)
   - NN sort time and travel saved
   - 2-Opt time and travel saved
   - Total optimized travel distance
   - Total pipeline execution time (from load to script transmission)
3. Calculate optimization percentage:
   - Formula: (1 - optimized_travel / raw_travel) × 100
4. Log Polyscope execution time:
   - Start clock when "draw_face()" begins
   - Stop when robot returns to home position
   - Record total elapsed time

**Pass/Fail Criteria:**
- ✓ PASS:
  - Travel reduction ≥ 20% (target: 20-30%)
  - NN sort completes in < 5 ms
  - 2-Opt refinement completes in < 100 ms
  - Total pipeline execution < 100 ms
  - Polyscope drawing execution < 2:30 minutes (150 seconds)
- ✗ FAIL:
  - Travel reduction < 15%
  - Any optimization step exceeds time budget
  - Polyscope execution exceeds 2:50 minutes
  - Inconsistent timing between runs (±5% variance acceptable)

**Evidence:**
- 3 consecutive test runs with timing logs
- Console output showing: raw travel, NN saved, 2-Opt saved, optimized travel
- Calculation spreadsheet: raw vs optimized comparison
- Polyscope timer screenshot / video timestamp

---

## Progress Summary

| Requirement | Status | Evidence |
|---|---|---|
| Trajectory generation from vectors | ✓ Complete | Code: `build_urscript()` function, tested with face1.json |
| Pen-up/pen-down motion | ✓ Complete | URScript movel() commands with Z_DRAW/Z_TRAVEL heights |
| Workspace validation | ✓ Complete | `validate_pose()` function, automatic scaling feature |
| Safe motion (no IK errors) | ✓ Complete | Tested on Polyscope, 0 E-stop triggers in 3 runs |
| <2:45 execution | ✓ Complete | Clocking 0.059-0.090s pipeline + ~120s Polyscope execution |
| Travel optimization | ✓ Complete | NN + 2-Opt achieving 30-41% reduction (exceeds 20-30% target) |
| Multi-run stability | ✓ Complete | Consistent output across 5 test executions |

**Subsystem Completion: ~90% toward Pass level**

---

## What's Working

✓ Full pipeline operational end-to-end  
✓ Automatic workspace scaling handles any input size  
✓ Pose validation prevents unreachable coordinates  
✓ Optimization consistently exceeds targets  
✓ Polyscope integration stable and reliable  
✓ Code clean and maintainable (no technical debt)

---

## What's Not Yet Complete

✗ Real UR3 hardware testing (simulator only)  
✗ ROS 2 driver integration (using direct TCP instead)  
✗ MoveIt2 integration (design choice: URScript more suitable for drawing)  
✗ Multi-color marker support (HD-level requirement)  
✗ Trajectory smoothing refinements (C-level requirement, partial)  
✗ Accuracy validation on real hardware (target: 5mm tolerance)

---

## Integration Plan & Next Steps

### Immediate (Next 2 weeks):
1. **Request approval** from coach on URScript vs MoveIt2 approach
2. **Real hardware testing** when UR3 becomes available
3. **Integrate ROS2 wrapper** (optional, for system compliance)
4. **Calibrate canvas origin** on physical table

### Short-term (Sprint 3):
1. Collaborate with Perception team to test with live camera feed
2. Test with Interaction team's GUI for end-to-end system
3. Implement trajectory smoothing (poly interpolation between waypoints)
4. Validate accuracy on physical drawings (measure deviation vs digital path)

### Longer-term (HD/Perfect levels):
1. Multi-color end-effector support
2. Dynamic angle adjustment for line thickness
3. Advanced shading techniques (cross-hatching optimization)

### Dependencies:
- Perception subsystem must output valid JSON strokes
- Interaction subsystem GUI integration for input parameters
- Actual UR3 access for hardware validation

### Risks:
- Real robot IK solver may differ from simulator (mitigation: margin testing)
- Network latency if ROS2 layer added (mitigation: keep TCP direct approach)
- Calibration drift affecting canvas origin (mitigation: quarterly re-calibration)

---

## Presentation Notes

When presenting, emphasize:
1. **Engineering trade-off:** URScript chosen over MoveIt2 because drawing environment is collision-free and deterministic behavior is critical
2. **Quantified results:** 30-41% travel reduction, 0.06s pipeline, <2:30 robot execution
3. **Readiness:** All Pass-level functional requirements complete; 90% ready for integration
4. **Clear path forward:** Awaiting real hardware and Perception output to proceed

