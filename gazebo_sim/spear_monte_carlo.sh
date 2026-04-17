#!/bin/bash
# ============================================================
# SPEAR Monte Carlo Harness v1.0
# Runs N single-pass intercept trials, logs results to CSV.
#
# Usage:
#   ./spear_monte_carlo.sh              # 50 trials, default config
#   ./spear_monte_carlo.sh 10           # 10 trials
#   ./spear_monte_carlo.sh 50 results   # 50 trials, output to results.csv
#
# Requirements:
#   - All 5 SPEAR processes must work individually
#   - Gazebo Fortress (ign gazebo)
#   - ROS2 Humble sourced
# ============================================================

set -e

NUM_TRIALS=${1:-50}
OUTPUT_PREFIX=${2:-"mc_$(date +%Y%m%d_%H%M%S)"}
OUTPUT_CSV="${OUTPUT_PREFIX}.csv"
LOG_DIR="mc_logs_${OUTPUT_PREFIX}"

SIM_DIR="$HOME/autonomous-uav-research/gazebo_sim"
WORLD_FILE="$HOME/autonomous-uav-research/gazebo_sim/two_drone_world.sdf"
BRIDGE_SCRIPT="$SIM_DIR/spear_camera_bridge.sh"
YOLO_SCRIPT="$SIM_DIR/spear_yolo_eyes_v2.py"
ACOUSTIC_SCRIPT="$SIM_DIR/spear_acoustic_cuer.py"
BRIDGE_V3="$SIM_DIR/spear_ros2_bridge_v3.py"

# Trial timeout — bridge has its own 60s timeout, but we add margin
TRIAL_TIMEOUT=90
# Time to wait for Gazebo to start
GAZEBO_STARTUP=10
# Time to wait for camera bridge + YOLO + acoustic to initialize
SENSOR_STARTUP=10

echo "============================================================"
echo "  SPEAR Monte Carlo Harness v1.0"
echo "  Trials: $NUM_TRIALS"
echo "  Output: $OUTPUT_CSV"
echo "  Logs:   $LOG_DIR/"
echo "============================================================"

# Create output directory
mkdir -p "$SIM_DIR/$LOG_DIR"
LOG_DIR="$SIM_DIR/$LOG_DIR"

# Write CSV header
OUTPUT_CSV="$SIM_DIR/$OUTPUT_CSV"
echo "trial,result,time_to_hit,min_sep,launch_err,vision_count,acoustic_count,elapsed" > "$OUTPUT_CSV"

# Clean kill function
kill_all() {
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "ruby.*ign" 2>/dev/null || true
    pkill -9 -f "spear_yolo" 2>/dev/null || true
    pkill -9 -f "spear_acoustic" 2>/dev/null || true
    pkill -9 -f "spear_ros2_bridge" 2>/dev/null || true
    pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
    pkill -9 -f "spear_camera_bridge" 2>/dev/null || true
    sleep 2
}

# Parse bridge output for results
parse_result() {
    local logfile="$1"
    local result="UNKNOWN"
    local tth="-1"
    local min_sep="-1"
    local launch_err="-1"
    local vis="-1"
    local acc="-1"

    # Extract launch error
    local launch_line=$(grep "LAUNCH BEARING" "$logfile" | tail -1)
    if [ -n "$launch_line" ]; then
        launch_err=$(echo "$launch_line" | grep -oP 'err=\K[0-9.]+')
    fi

    # Extract HIT or MISS line
    local hit_line=$(grep '^\[INFO\].*\*\*\* HIT' "$logfile" | head -1)
    local miss_line=$(grep '^\[WARN\].*\*\*\* MISS' "$logfile" | head -1)

    if [ -n "$hit_line" ]; then
        result="HIT"
        tth=$(echo "$hit_line" | grep -oP 't=\K[0-9.]+')
        min_sep=$(echo "$hit_line" | grep -oP 'min_sep=\K[0-9.]+')
        vis=$(echo "$hit_line" | grep -oP 'vis=\K[0-9]+')
        acc=$(echo "$hit_line" | grep -oP 'acc=\K[0-9]+')
    elif [ -n "$miss_line" ]; then
        result=$(echo "$miss_line" | grep -oP '\*\*\* \K[A-Z-]+')
        min_sep=$(echo "$miss_line" | grep -oP 'min_sep=\K[0-9.]+')
        vis=$(echo "$miss_line" | grep -oP 'vis=\K[0-9]+')
        acc=$(echo "$miss_line" | grep -oP 'acc=\K[0-9]+')
    fi

    echo "${result},${tth},${min_sep},${launch_err},${vis},${acc}"
}

# Trap Ctrl+C to clean up
trap 'echo ""; echo "Interrupted. Cleaning up..."; kill_all; echo "Results so far saved to $OUTPUT_CSV"; exit 1' INT

# ============ MAIN LOOP ============
hits=0
misses=0
errors=0
total_tth=0

for trial in $(seq 1 $NUM_TRIALS); do
    echo ""
    echo "--- Trial $trial/$NUM_TRIALS ---"

    # Kill everything from previous trial
    kill_all

    trial_log="$LOG_DIR/trial_${trial}.log"
    trial_start=$(date +%s)

    # 1. Start Gazebo (headless for speed)
    echo "  Starting Gazebo..."
    cd "$HOME/autonomous-uav-research"
    ign gazebo --headless-rendering -r "$WORLD_FILE" > "$LOG_DIR/gazebo_${trial}.log" 2>&1 &
    GAZEBO_PID=$!
    sleep $GAZEBO_STARTUP

    # Check Gazebo is alive
    if ! kill -0 $GAZEBO_PID 2>/dev/null; then
        echo "  ERROR: Gazebo failed to start"
        echo "${trial},ERROR-GAZEBO,-1,-1,-1,-1,-1,-1" >> "$OUTPUT_CSV"
        errors=$((errors + 1))
        continue
    fi

    # 2. Start camera bridge
    echo "  Starting camera bridge..."
    cd "$SIM_DIR"
    bash "$BRIDGE_SCRIPT" > "$LOG_DIR/camera_${trial}.log" 2>&1 &

    # 3. Start YOLO
    echo "  Starting YOLO..."
    python3 "$YOLO_SCRIPT" > "$LOG_DIR/yolo_${trial}.log" 2>&1 &

    # 4. Start acoustic cuer
    echo "  Starting acoustic cuer..."
    python3 "$ACOUSTIC_SCRIPT" > "$LOG_DIR/acoustic_${trial}.log" 2>&1 &

    # Wait for sensors to initialize
    sleep $SENSOR_STARTUP

    # 5. Run the bridge (main trial)
    echo "  Running intercept trial..."
    timeout $TRIAL_TIMEOUT python3 "$BRIDGE_V3" --vision --timeout 60 \
        > "$trial_log" 2>&1 || true

    trial_end=$(date +%s)
    elapsed=$((trial_end - trial_start))

    # Parse results
    parsed=$(parse_result "$trial_log")
    result=$(echo "$parsed" | cut -d',' -f1)
    tth=$(echo "$parsed" | cut -d',' -f2)

    # Write CSV row
    echo "${trial},${parsed},${elapsed}" >> "$OUTPUT_CSV"

    # Update counters
    if [ "$result" = "HIT" ]; then
        hits=$((hits + 1))
        if [ "$tth" != "-1" ]; then
            # Bash can't do float math, use python
            total_tth=$(python3 -c "print(${total_tth} + ${tth})")
        fi
        echo "  >> HIT  t=${tth}s  (${hits}/${trial} = $(python3 -c "print(f'{${hits}/${trial}*100:.0f}')")% hit rate)"
    elif echo "$result" | grep -q "MISS"; then
        misses=$((misses + 1))
        echo "  >> MISS  (${hits}/${trial} = $(python3 -c "print(f'{${hits}/${trial}*100:.0f}')")% hit rate)"
    else
        errors=$((errors + 1))
        echo "  >> ERROR: $result"
    fi
done

# ============ FINAL CLEANUP ============
kill_all

# ============ SUMMARY ============
echo ""
echo "============================================================"
echo "  MONTE CARLO COMPLETE"
echo "============================================================"
echo "  Trials:    $NUM_TRIALS"
echo "  Hits:      $hits"
echo "  Misses:    $misses"
echo "  Errors:    $errors"
if [ $NUM_TRIALS -gt 0 ]; then
    echo "  Hit rate:  $(python3 -c "print(f'{${hits}/${NUM_TRIALS}*100:.1f}%')")"
fi
if [ $hits -gt 0 ]; then
    echo "  Avg TTH:   $(python3 -c "print(f'{${total_tth}/${hits}:.1f}s')")"
fi
echo "  Output:    $OUTPUT_CSV"
echo "  Logs:      $LOG_DIR/"
echo "============================================================"
echo ""
echo "View results:"
echo "  cat $OUTPUT_CSV"
echo "  column -t -s',' $OUTPUT_CSV"
