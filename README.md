# Autonomous UAV Intercept Simulation

ArduPilot SITL simulation for autonomous navigation and intercept of moving virtual targets.

## The Problem

MAVLink `SET_POSITION_TARGET_GLOBAL_INT` velocity commands are ignored by ArduCopter's position controller in GUIDED mode — the drone reports 0 m/s groundspeed and cannot close distance below ~22 m to a moving waypoint.

## The Fix: Overshoot Navigation

Instead of commanding velocities, we use `simple_goto` aimed at a point **50 m past** the predicted target position. This means:

- The flight controller always has a distant waypoint ahead and never triggers deceleration.
- The drone flies *through* the target coordinates at full cruise speed.
- Contact is registered when the drone passes within 10 m of the target.

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  Guidance Loop                   │
│  (runs every 0.25 s)                            │
│                                                  │
│  1. Update all SimTarget positions               │
│  2. Select highest-priority target (nearest tie) │
│  3. Estimate time-to-intercept                   │
│  4. Predict target position at intercept time    │
│  5. Apply ProNav bearing correction              │
│  6. Project overshoot point 50 m past prediction │
│  7. Issue simple_goto to overshoot point         │
│  8. Check contact (< 10 m → intercept)           │
└─────────────────────────────────────────────────┘
```

### Proportional Navigation (ProNav)

The bearing to the aim point is corrected by the line-of-sight (LOS) rate:

```
commanded_bearing = LOS_bearing + N × LOS_rate × dt
```

where **N = 3** (navigation gain). This biases the flight path toward the future intercept point, reducing the pursuit curve.

### Priority-Based Target Selection

Targets carry an integer priority (lower = more important). The selector picks:
1. Lowest priority number first.
2. Among equal priorities, the nearest target wins.

Once a target is intercepted (distance < 10 m), it is removed and the next target is selected.

## Files

| File | Purpose |
|---|---|
| `intercept_sim.py` | Main simulation script (guidance, nav, targets) |
| `intercept_params.parm` | Custom ArduPilot parameters for fast navigation |
| `launch_sim.sh` | One-command launcher: starts SITL + runs script |

## Quick Start

```bash
# 1. Make sure ArduPilot SITL tools are on PATH
source ~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh

# 2. Launch everything
chmod +x launch_sim.sh
bash launch_sim.sh
```

Or run the pieces separately:

```bash
# Terminal 1 — SITL
sim_vehicle.py -v ArduCopter --no-mavproxy \
    --add-param-file=intercept_params.parm

# Terminal 2 — intercept script
python3 intercept_sim.py --connect tcp:127.0.0.1:5762
```

## Key Parameters (intercept_params.parm)

| Parameter | Value | Why |
|---|---|---|
| `WPNAV_SPEED` | 1500 (15 m/s) | High cruise speed for intercept |
| `WPNAV_ACCEL` | 400 (4 m/s²) | Fast acceleration to cruise |
| `WPNAV_RADIUS` | 200 (2 m) | Tight acceptance radius |
| `LOIT_BRK_DELAY` | 0.1 s | Near-instant braking when needed |
| `THR_MIN` | 130 | Maintains speed through turns |

## Tuning

- **OVERSHOOT_DISTANCE_M** (default 50): Increase if the drone still decelerates before contact. Decrease for tighter turning at the cost of possible premature braking.
- **CONTACT_THRESHOLD_M** (default 10): The "kill radius." Smaller values require more precise guidance.
- **NAV_GAIN** (default 3.0): ProNav constant N. Higher values steer more aggressively toward the predicted intercept. Values above 4-5 can cause oscillation.
- **COMMAND_INTERVAL_S** (default 0.25): How often the guidance loop re-aims. Faster updates improve tracking but add MAVLink traffic.
