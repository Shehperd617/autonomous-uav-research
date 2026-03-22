#!/usr/bin/env bash
# ============================================================
# launch_sim.sh — Start ArduCopter SITL + intercept script
# Usage:  bash launch_sim.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PARAM_FILE="${SCRIPT_DIR}/intercept_params.parm"
SIM_SCRIPT="${SCRIPT_DIR}/interceptor_v9.py"
CONN_STRING="tcp:127.0.0.1:5762"

# ---------- Preflight checks ----------
command -v sim_vehicle.py >/dev/null 2>&1 || {
    echo "ERROR: sim_vehicle.py not found."
    echo "       Make sure ArduPilot SITL tools are on your PATH."
    echo "       (source Tools/environment_install/install-prereqs-ubuntu.sh)"
    exit 1
}

command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 not found."; exit 1; }
python3 -c "import dronekit" 2>/dev/null || {
    echo "Installing dronekit …"
    pip3 install dronekit dronekit-sitl --break-system-packages
}

# ---------- Start SITL in background ----------
echo "============================================"
echo " Launching ArduCopter SITL"
echo " Param file: ${PARAM_FILE}"
echo "============================================"

sim_vehicle.py \
    -v ArduCopter \
    --no-mavproxy \
    --add-param-file="${PARAM_FILE}" \
    -I 0 &
SIM_PID=$!

# Give SITL time to boot and bind its TCP port
echo "Waiting 15 s for SITL to initialise …"
sleep 15

# ---------- Run intercept script ----------
echo ""
echo "============================================"
echo " Starting intercept simulation"
echo " Connection: ${CONN_STRING}"
echo "============================================"
echo ""

python3 "${SIM_SCRIPT}" --connect "${CONN_STRING}"
STATUS=$?

# ---------- Cleanup ----------
echo ""
echo "Shutting down SITL (PID ${SIM_PID}) …"
kill "${SIM_PID}" 2>/dev/null || true
wait "${SIM_PID}" 2>/dev/null || true

exit ${STATUS}
