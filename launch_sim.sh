#!/usr/bin/env bash
# ============================================================
# launch_sim.sh — Start ArduCopter SITL + intercept script
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PARAM_FILE="${SCRIPT_DIR}/intercept_params.parm"
SIM_SCRIPT="${SCRIPT_DIR}/interceptor_v9.py"
CONN_STRING="tcp:127.0.0.1:5760"

# ---------- Kill leftover SITL ----------
echo "Cleaning up old processes …"
pkill -f sim_vehicle.py 2>/dev/null || true
pkill -f arducopter 2>/dev/null || true
sleep 2

# ---------- Preflight ----------
command -v sim_vehicle.py >/dev/null 2>&1 || {
    echo "ERROR: sim_vehicle.py not on PATH"; exit 1; }
python3 -c "import dronekit" 2>/dev/null || {
    pip3 install dronekit --break-system-packages; }

# ---------- Start SITL ----------
echo "============================================"
echo " Launching ArduCopter SITL"
echo " Params: ${PARAM_FILE}"
echo " Using -w flag (wipe) for clean param load"
echo "============================================"

sim_vehicle.py \
    -v ArduCopter \
    --no-mavproxy \
    --add-param-file="${PARAM_FILE}" \
    -w \
    -I 0 &
SIM_PID=$!

echo "Waiting 40 s for SITL …"
sleep 40

# ---------- Run script ----------
echo ""
echo "============================================"
echo " Starting intercept — ${CONN_STRING}"
echo "============================================"
echo ""

python3 "${SIM_SCRIPT}" --connect "${CONN_STRING}"
STATUS=$?

# ---------- Cleanup ----------
kill "${SIM_PID}" 2>/dev/null || true
wait "${SIM_PID}" 2>/dev/null || true
exit ${STATUS}
