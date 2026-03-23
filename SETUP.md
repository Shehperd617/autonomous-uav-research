# How to Run the Interceptor (Beginner Guide)

Everything runs in simulation. No real drone needed.

## Step 1: Install Ubuntu (WSL)

If you're on Windows, open PowerShell as admin and run:

```
wsl --install -d Ubuntu
```

Restart your PC, then open Ubuntu from the Start menu. Set a username and password when asked.

## Step 2: Install ArduPilot SITL

Run these one at a time in your Ubuntu terminal:

```
sudo apt update && sudo apt install -y git
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Close and reopen your terminal, then:

```
. ~/.profile
```

## Step 3: Install Python Dependencies

```
pip install dronekit pymavlink
```

## Step 4: Get the Interceptor Script

```
git clone https://github.com/Shehperd617/autonomous-uav-research.git ~/autonomous-uav-research
```

## Step 5: Run It

You need two terminals open. Both should be Ubuntu.

**Terminal 1 — Start the simulator:**

```
sim_vehicle.py -v ArduCopter -f JSON:Tools/autotest/models/freestyle.json --out=udp:127.0.0.1:14560 -w
```

Wait until you see `APM: EKF3 IMU0 is using GPS` or similar messages settle down (about 30 seconds).

**Terminal 2 — Run the interceptor:**

```
python3 ~/autonomous-uav-research/interceptor_v11.py --connect udp:127.0.0.1:14560
```

That's it. You should see the drone arm, take off, and start chasing targets.

## What You'll See

The output shows the drone chasing 3 simulated targets:

```
T#1 [PRONAV ] d= 99.8m  GS= 19.4  cmd= 20.0  pk=19.4
```

- `T#1` — which target it's chasing
- `PRONAV / PURSUIT / RAM` — guidance phase
- `d=` — distance to target in meters
- `GS=` — groundspeed in m/s
- `cmd=` — commanded speed
- `pk=` — peak speed so far

When a target is intercepted you'll see:

```
*** INTERCEPT #1 d=7.1m GS=16.8m/s t=12.6s ***
```

The mission ends when all 3 targets are caught.

## If Something Goes Wrong

**"Connection refused"** — Terminal 1 isn't ready yet. Wait longer before running Terminal 2.

**Drone won't arm** — Kill everything and restart with a clean state:

```
pkill -f arducopter; pkill -f mavproxy; pkill -f sim_vehicle
```

Then start Terminal 1 again with the `-w` flag (wipes saved params).

**"No module named dronekit"** — Run `pip install dronekit pymavlink` again.

## Expected Results (v11)

| Target | Speed | Intercept Time |
|--------|-------|----------------|
| T#1 | 8 m/s | ~12s |
| T#2 | 5 m/s | ~34s |
| T#3 | 10 m/s | ~118s |

Peak drone speed: ~20 m/s. Total mission: ~2 minutes. All 3 targets intercepted, zero oscillation.
