#!/bin/bash

# Test 1 Suite Runner: 5 runs for each face (face1, face2, face3)
# Motion planning with collision avoidance

set -e

LOG_DIR="$HOME/RS2/test_results"
ROS_WS="$HOME/RS2/ros2_ws"
mkdir -p "$LOG_DIR"

echo "=================================================================="
echo "TEST 1 SUITE - MOTION PLANNING WITH COLLISION AVOIDANCE"
echo "Running: 5 runs × 3 faces = 15 total tests"
echo "=================================================================="
echo ""

source "$ROS_WS/install/setup.bash"
cd "$ROS_WS"

FACES=("face1" "face2" "face3")
RUNS=5
test_num=0
total_tests=$((RUNS * ${#FACES[@]}))

for face in "${FACES[@]}"; do
    echo "========== ${face^^} =========="
    for run in $(seq 1 $RUNS); do
        ((test_num++))
        log_file="$LOG_DIR/test1_${face}_run${run}.log"
        
        printf "[%2d/%2d] Run %d: " "$test_num" "$total_tests" "$run"
        
        # Run with timeout
        if timeout 120 ros2 run ur3_motion_planning motion_planning_node \
            --ros-args -p robot_ip:=192.168.56.101 -p face:="$face" \
            > "$log_file" 2>&1; then
            
            # Check if successful
            if grep -q "TRAJECTORY EXECUTED SUCCESSFULLY" "$log_file"; then
                strokes=$(grep -oP 'Loaded \K\d+' "$log_file" | head -1)
                waypoints=$(grep -oP '\K\d+(?= Cartesian waypoints)' "$log_file" | head -1)
                echo "✓ PASS | Strokes: $strokes, Waypoints: $waypoints"
            else
                echo "✗ FAIL (no success)"
            fi
        else
            echo "✗ FAIL (timeout/error)"
        fi
        
        sleep 1
    done
    echo ""
done

echo "=================================================================="
echo "TEST SUITE COMPLETE"
echo "=================================================================="
echo "Logs saved to: $LOG_DIR/"
echo "Generated log files:"
ls -lh "$LOG_DIR"/test1_*.log 2>/dev/null | tail -5
echo "..."
echo ""
echo "Total: $((${#FACES[@]} * RUNS)) tests completed"
