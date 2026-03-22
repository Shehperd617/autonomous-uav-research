#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PARAM_FILE="${SCRIPT_DIR}/intercept_params.parm"
SIM_SCRIPT="${SCRIPT_DIR}/interceptor_v9.py"
CONN_STRING="tcp:127.0.0.1:5760"

echo "Cleaning up old processes …"
pkill -f sim_vehicle.py 2>/dev/null || true
pkill -f arducopter 2>/dev/null || true
sleep 2

command -v sim_vehicle.py >/dev/null 2>&1 || { echo "ERROR: sim_vehicle.py not on PATH"; exit 1; }
python3 -c "import dronekit" 2>/dev/null || { pip3 install dronekit --break-system-packages; }

echo "============================================"
echo " Launching ArduCopter SITL (wipe + params)"
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

echo ""
echo "============================================"
echo " Starting intercept — ${CONN_STRING}"
echo "============================================"
echo ""

python3 "${SIM_SCRIPT}" --connect "${CONN_STRING}"
STATUS=$?

kill "${SIM_PID}" 2>/dev/null || true
wait "${SIM_PID}" 2>/dev/null || true
exit ${STATUS}
