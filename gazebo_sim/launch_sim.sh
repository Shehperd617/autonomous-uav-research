#!/bin/bash
# SPEAR Gazebo Simulation Launcher
# Usage: ./launch_sim.sh

echo "============================================"
echo "  SPEAR Two-Drone Intercept Simulation"
echo "============================================"
echo ""

# Ensure render settings
export LIBGL_ALWAYS_SOFTWARE=1
export IGN_GAZEBO_RENDER_ENGINE_GUI=ogre
export IGN_GAZEBO_RENDER_ENGINE_SERVER=ogre

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Step 1: Starting Gazebo..."
echo "  Running: ign gazebo $SCRIPT_DIR/two_drone_world.sdf"
echo ""
echo "  Once Gazebo opens and you see the two drones:"
echo "    - Blue drone = interceptor (SPEAR)"
echo "    - Red drone = target (threat)"
echo "    - Press the PLAY button (orange ▶ bottom-left)"
echo ""
echo "  Then in a NEW terminal, run:"
echo "    cd $SCRIPT_DIR"
echo "    python3 spear_gazebo_bridge.py"
echo ""

ign gazebo "$SCRIPT_DIR/two_drone_world.sdf"
