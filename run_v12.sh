#!/bin/bash
# run_v12.sh — Rebuilds SITL with modified freestyle.json, then launches interceptor
set -e

echo "=== INTERCEPTOR v12 LAUNCHER ==="

# 1. Kill everything
echo "[1/5] Cleaning up..."
pkill -9 -f arducopter 2>/dev/null || true
pkill -9 -f mavproxy 2>/dev/null || true
pkill -9 -f sim_vehicle 2>/dev/null || true
sleep 2

# 2. Modify freestyle.json for 40+ m/s
echo "[2/5] Patching freestyle.json for high speed..."
FREESTYLE="$HOME/ardupilot/Tools/autotest/models/freestyle.json"

# Backup if not already done
[ ! -f "${FREESTYLE}.stock" ] && cp "$FREESTYLE" "${FREESTYLE}.stock"

# Write the high-speed model
cat > "$FREESTYLE" << 'JSON'
# High-speed interceptor model (modified freestyle)
{
    "mass" : 0.8,
    "diagonal_size" : 0.25,
    "refSpd" : 45.0,
    "refAngle" : 65.0,
    "refVoltage" : 23.2,
    "refCurrent" : 12,
    "refAlt" : 607,
    "refTempC" : 25,
    "refBatRes" : 0.0226,
    "maxVoltage" : 25.2,
    "battCapacityAh" : 0,
    "propExpo" : 0.5,
    "refRotRate" : 900,
    "hoverThrOut" : 0.04,
    "pwmMin" : 1000,
    "pwmMax" : 2000,
    "spin_min" : 0.01,
    "spin_max" : 0.95,
    "slew_max" : 0,
    "disc_area" : 0.35,
    "mdrag_coef" : 0.03,
    "num_motors" : 4
}
JSON

# 3. Force SITL rebuild (re-embeds the JSON)
echo "[3/5] Rebuilding SITL (embeds new JSON)..."
cd $HOME/ardupilot
./waf build --target bin/arducopter 2>&1 | tail -5

# 4. Launch SITL with freestyle model
echo "[4/5] Launching SITL with freestyle model..."
sim_vehicle.py -v ArduCopter --model quad --frame freestyle \
  --out=udp:127.0.0.1:14560 -w --no-rebuild &

# 5. Wait for SITL boot then launch interceptor
echo "[5/5] Waiting 15s for SITL to boot..."
sleep 15

echo "=== LAUNCHING INTERCEPTOR v12 ==="
python3 $HOME/autonomous-uav-research/interceptor_v12.py --connect udp:127.0.0.1:14560
