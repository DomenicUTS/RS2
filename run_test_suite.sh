#!/bin/bash

# Test Suite Runner: 5 runs for each face (1, 2, 3)
# Logs: test_run_face{N}_{RUN}.log

echo "=================================================="
echo "UR3 MOTION PLANNING - TEST SUITE"
echo "Running 5 test cycles for Face 1, 2, and 3"
echo "=================================================="
echo ""

FACES=(1 2 3)
RUNS=5

for FACE in "${FACES[@]}"; do
    echo "========== FACE $FACE =========="
    for RUN in $(seq 1 $RUNS); do
        echo "  Run $RUN of $RUNS..."
        python3 src/ur3_selfie_draw.py $FACE $RUN > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "    ✓ Test complete (log: test_run_face${FACE}_${RUN}.log)"
        else
            echo "    ✗ Test failed"
        fi
    done
    echo ""
done

echo "=================================================="
echo "TEST SUITE COMPLETE"
echo "=================================================="
echo ""
echo "Generated log files:"
ls -lh test_run_*.log 2>/dev/null || echo "No log files found"
echo ""
echo "Summary:"
echo "  Total tests: $((${#FACES[@]} * RUNS))"
echo "  Per face: $RUNS runs"
