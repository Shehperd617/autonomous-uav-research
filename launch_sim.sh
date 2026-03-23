#!/usr/bin/env bash
set -euo pipefail

# ═══════════════════════════════════════════════════════════════
# launch_sim.sh — ArduCopter SITL + Interceptor v10
#
# Tries freestyle JSON model first (30-40 m/s capable).
# Falls back to default "+" quad (~20-27 m/s with vel cmds).
# DISPLAY= fixes xterm crash on WSL.
# ═══════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PARAM_FILE="${SCRIPT_DIR}/intercept_params.parm"
SIM_SCRIPT="${SCRIPT_DIR}/interceptor_v10.py"
CONN_STRING="tcp:127.0.0.1:5760"

# Freestyle model paths (ArduPilot source tree)
FREESTYLE_JSON="${ARDUPILOT_HOME:-$HOME/ardupilot}/Tools/autotest/models/freestyle.json"
FREESTYLE_PARAM="${ARDUPILOT_HOME:-$HOME/ardupilot}/Tools/autotest/models/freestyle.param"

# ── Cleanup ──────────────────────────────────────────────────
echo "Cleaning up old SITL processes ..."
pkill -f sim_vehicle.py  2>/dev/null || true
pkill -f arducopter      2>/dev/null || true
sleep 3

# ── Dependency check ─────────────────────────────────────────
command -v sim_vehicle.py >/dev/null 2>&1 || {
    echo "ERROR: sim_vehicle.py not on PATH"
    echo "  Source: ~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh"
    echo "  Then:   . ~/.profile"
    exit 1
}

python3 -c "from pymavlink import mavutil" 2>/dev/null || {
    echo "Installing pymavlink ..."
    pip3 install pymavlink --break-system-packages
}

# ── Check for dronekit (v10 doesn't need it, but warn if missing for v9) ──
if ! python3 -c "import dronekit" 2>/dev/null; then
    echo "NOTE: dronekit not installed (v10 uses pymavlink directly — OK)"
fi

# ── Select model ─────────────────────────────────────────────
SIM_ARGS=()

if [[ -f "${FREESTYLE_JSON}" && -f "${FREESTYLE_PARAM}" ]]; then
    echo "============================================"
    echo " Freestyle JSON model FOUND"
    echo "   ${FREESTYLE_JSON}"
    echo " Model: 0.8 kg racing quad"
    echo " Speed capability: 30-40 m/s"
    echo "============================================"
    SIM_ARGS+=(
        --model "JSON:${FREESTYLE_JSON}"
        "--add-param-file=${FREESTYLE_PARAM}"
    )
    MODEL_DESC="Freestyle racing quad (JSON)"
    BOOT_WAIT=60
else
    echo "============================================"
    echo " Freestyle JSON model NOT FOUND at:"
    echo "   ${FREESTYLE_JSON}"
    echo ""
    echo " Using default '+' quad model"
    echo " Speed capability: ~20-27 m/s with vel cmds"
    echo " (still WAY better than v9's 14 m/s)"
    echo ""
    echo " To get 30-40 m/s, create freestyle.json in:"
    echo "   ~/ardupilot/Tools/autotest/models/"
    echo "============================================"
    MODEL_DESC="Default + quad (velocity NED)"
    BOOT_WAIT=30
fi

# ── Launch SITL ──────────────────────────────────────────────
echo ""
echo "Launching ArduCopter SITL ..."
echo "  Model: ${MODEL_DESC}"
echo "  Params: ${PARAM_FILE}"
echo "  DISPLAY= (no xterm — WSL safe)"
echo ""

# DISPLAY= prevents xterm from spawning (crashes on WSL without X server)
# --no-mavproxy avoids second terminal dependency
DISPLAY= sim_vehicle.py \
    -v ArduCopter \
    --no-mavproxy \
    -w \
    -I 0 \
    "${SIM_ARGS[@]}" \
    "--add-param-file=${PARAM_FILE}" &
SIM_PID=$!

echo "Waiting ${BOOT_WAIT}s for SITL boot ..."
echo "(Freestyle JSON takes longer due to model loading)"
sleep "${BOOT_WAIT}"

# Verify SITL is running
if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "ERROR: SITL process died. Check ArduPilot build."
    echo "  Try: cd ~/ardupilot && ./waf configure --board sitl && ./waf copter"
    exit 1
fi

# ── Launch interceptor ───────────────────────────────────────
echo ""
echo "============================================"
echo " INTERCEPTOR v10 — HYBRID GUIDANCE"
echo " PRONAV (>50m) → PURSUIT (15-50m) → RAM (<15m)"
echo " Velocity NED commands — no WP planner speed cap"
echo " Connection: ${CONN_STRING}"
echo "============================================"
echo ""

python3 "${SIM_SCRIPT}" --connect "${CONN_STRING}"
STATUS=$?

# ── Cleanup ──────────────────────────────────────────────────
echo ""
echo "Shutting down SITL ..."
kill "${SIM_PID}" 2>/dev/null || true
wait "${SIM_PID}" 2>/dev/null || true

if [[ ${STATUS} -eq 0 ]]; then
    echo "Mission complete. Log: intercept_v10_log.csv"
else
    echo "Mission exited with status ${STATUS}"
fi

exit ${STATUS}
