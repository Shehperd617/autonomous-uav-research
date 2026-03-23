#!/usr/bin/env python3
"""
Interceptor v9 — Autonomous UAV Intercept Simulation (ArduPilot SITL)
======================================================================
40 m/s interceptor drone vs moving targets.

Built for ArduPilot master (2025+) which uses:
  ATC_ANGLE_MAX (degrees)  not ANGLE_MAX (centidegrees)
  LOIT_SPEED_MS (m/s)      not WPNAV_SPEED (cm/s)
  LOIT_ACC_MAX_M (m/s²)    not WPNAV_ACCEL (cm/s²)

The key to high speed: ATC_ANGLE_MAX = 60° (default 30° caps at ~10 m/s)
"""

from __future__ import annotations

import math
import time
import argparse
import signal
import sys
from dataclasses import dataclass, field
from typing import List, Optional

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
OVERSHOOT_DISTANCE_M: float = 30.0
CONTACT_THRESHOLD_M: float = 15.0
COMMAND_INTERVAL_S: float = 0.20
DEFAULT_ALT_M: float = 30.0
EARTH_RADIUS_M: float = 6_371_000.0
NAV_GAIN: float = 3.0

# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def haversine_m(lat1, lon1, lat2, lon2):
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlat, dlon = rlat2 - rlat1, rlon2 - rlon1
    a = (math.sin(dlat/2)**2
         + math.cos(rlat1)*math.cos(rlat2)*math.sin(dlon/2)**2)
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def bearing_deg(lat1, lon1, lat2, lon2):
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = (math.cos(rlat1)*math.sin(rlat2)
         - math.sin(rlat1)*math.cos(rlat2)*math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360

def destination_point(lat, lon, bearing_d, dist_m):
    rlat, rlon = math.radians(lat), math.radians(lon)
    rb = math.radians(bearing_d)
    d_r = dist_m / EARTH_RADIUS_M
    new_lat = math.asin(
        math.sin(rlat)*math.cos(d_r)
        + math.cos(rlat)*math.sin(d_r)*math.cos(rb))
    new_lon = rlon + math.atan2(
        math.sin(rb)*math.sin(d_r)*math.cos(rlat),
        math.cos(d_r) - math.sin(rlat)*math.sin(new_lat))
    return math.degrees(new_lat), math.degrees(new_lon)

# ---------------------------------------------------------------------------
# Target
# ---------------------------------------------------------------------------

@dataclass
class SimTarget:
    id: int
    lat: float
    lon: float
    alt: float
    course_deg: float
    speed_mps: float
    priority: int = 1
    _last_update: float = field(default_factory=time.time, repr=False)

    def step(self):
        now = time.time()
        dt = now - self._last_update
        self._last_update = now
        self.lat, self.lon = destination_point(
            self.lat, self.lon, self.course_deg, self.speed_mps * dt)

    def predicted_position(self, t_sec):
        return destination_point(
            self.lat, self.lon, self.course_deg, self.speed_mps * t_sec)

    def reset_clock(self):
        self._last_update = time.time()

# ---------------------------------------------------------------------------
# Target selector
# ---------------------------------------------------------------------------

def select_target(targets, drone_lat, drone_lon):
    if not targets:
        return None
    return min(targets, key=lambda t: (
        t.priority, haversine_m(drone_lat, drone_lon, t.lat, t.lon)))

# ---------------------------------------------------------------------------
# ProNav guidance
# ---------------------------------------------------------------------------

class ProNavGuidance:
    def __init__(self, gain=NAV_GAIN, overshoot_m=OVERSHOOT_DISTANCE_M):
        self.gain = gain
        self.overshoot_m = overshoot_m
        self._prev_los = None
        self._prev_time = None

    def compute(self, drone_lat, drone_lon, drone_speed, target):
        now = time.time()
        dist = haversine_m(drone_lat, drone_lon, target.lat, target.lon)
        closing = max(drone_speed, 1.0) + target.speed_mps
        t_int = dist / closing

        pred_lat, pred_lon = target.predicted_position(t_int)
        los = bearing_deg(drone_lat, drone_lon, pred_lat, pred_lon)

        if self._prev_los is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                d_los = (los - self._prev_los + 180) % 360 - 180
                los = (los + self.gain * (d_los / dt) * dt) % 360

        self._prev_los = los
        self._prev_time = now

        aim_lat, aim_lon = destination_point(
            pred_lat, pred_lon, los, self.overshoot_m)
        return LocationGlobalRelative(aim_lat, aim_lon, target.alt)

# ---------------------------------------------------------------------------
# Vehicle helpers
# ---------------------------------------------------------------------------

def set_param(vehicle, name, value):
    msg = vehicle.message_factory.param_set_encode(
        0, 0, name.encode("utf-8"), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def force_speed_params(vehicle):
    """Set correct params for ArduPilot master (2025+)."""
    params = {
        # THE KEY: max lean angle in DEGREES (not centidegrees!)
        "ATC_ANGLE_MAX":  60.0,     # 60° lean (default 30°)
        "ATC_ANGLE_BOOST": 0.0,     # don't waste thrust on tilt compensation
        "ATC_ACC_P_MAX":  2200.0,   # fast pitch rate
        "ATC_ACC_R_MAX":  2200.0,   # fast roll rate
        # Speed and acceleration in M/S (not cm/s!)
        "LOIT_SPEED_MS":  40.0,     # 40 m/s max speed
        "LOIT_ACC_MAX_M": 15.0,     # 15 m/s² acceleration
        "LOIT_BRK_ACC_M": 8.0,     # braking accel
        "LOIT_BRK_DELAY": 0.1,     # instant braking
        "LOIT_BRK_JRK_M": 15.0,    # braking jerk
        # Position controller
        "PSC_NE_VEL_P":   6.0,     # aggressive velocity tracking
        "PSC_NE_VEL_I":   2.0,
        "PSC_NE_VEL_D":   0.5,
        "PSC_NE_POS_P":   2.0,
        "PSC_JERK_NE":    20.0,
    }
    print("\n  Forcing speed params (ArduPilot master format):")
    for k, v in params.items():
        set_param(vehicle, k, v)
        print(f"    {k} = {v}")
        time.sleep(0.2)
    time.sleep(3)
    print(f"\n  ✓ ATC_ANGLE_MAX = 60° (was 30°)")
    print(f"  ✓ LOIT_SPEED_MS = 40 m/s (was 12.5)")
    print(f"  ✓ LOIT_ACC_MAX_M = 15 m/s²")
    print(f"  Target top speed: ~35-40 m/s\n")

def wait_armable(vehicle, timeout=120):
    print("  Waiting for armable …")
    t0 = time.time()
    while not vehicle.is_armable:
        if time.time() - t0 > timeout:
            raise TimeoutError("Not armable")
        time.sleep(0.5)
    print("  Armable.")

def arm_and_takeoff(vehicle, alt):
    wait_armable(vehicle)
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.3)
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.3)
    print("  Armed! Taking off …")
    vehicle.simple_takeoff(alt)
    while True:
        cur = vehicle.location.global_relative_frame.alt or 0
        if cur >= alt * 0.95:
            print(f"  Alt {cur:.1f} m — takeoff done.\n")
            break
        time.sleep(0.5)

# ---------------------------------------------------------------------------
# Build targets (close, spawned after takeoff)
# ---------------------------------------------------------------------------

def build_targets(lat, lon):
    return [
        SimTarget(id=1, priority=1,
                  lat=lat + 0.0008, lon=lon + 0.0005,
                  alt=DEFAULT_ALT_M, course_deg=45, speed_mps=8),
        SimTarget(id=2, priority=2,
                  lat=lat - 0.0005, lon=lon + 0.0010,
                  alt=DEFAULT_ALT_M, course_deg=120, speed_mps=5),
        SimTarget(id=3, priority=3,
                  lat=lat + 0.0012, lon=lon - 0.0003,
                  alt=DEFAULT_ALT_M, course_deg=315, speed_mps=10),
    ]

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(conn):
    print(f"Connecting to {conn} …")
    vehicle = connect(conn, wait_ready=True)
    force_speed_params(vehicle)

    def _sig(s, f):
        print("\n[SIGINT] Landing …")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(2); vehicle.close(); sys.exit(0)
    signal.signal(signal.SIGINT, _sig)

    arm_and_takeoff(vehicle, DEFAULT_ALT_M)

    loc = vehicle.location.global_relative_frame
    targets = build_targets(loc.lat, loc.lon)
    for t in targets:
        t.reset_clock()
        d = haversine_m(loc.lat, loc.lon, t.lat, t.lon)
        print(f"  Target #{t.id}: {d:.0f} m, speed {t.speed_mps} m/s, "
              f"hdg {t.course_deg}°")

    guidance = ProNavGuidance()
    active = list(targets)
    done = []
    peak_gs = 0.0

    print("\n══════════════════════════════════════")
    print("  INTERCEPT MISSION STARTED")
    print("══════════════════════════════════════\n")

    while active:
        loc = vehicle.location.global_relative_frame
        gs = vehicle.groundspeed or 0.0
        peak_gs = max(peak_gs, gs)

        for t in active:
            t.step()

        tgt = select_target(active, loc.lat, loc.lon)
        if not tgt:
            break

        dist = haversine_m(loc.lat, loc.lon, tgt.lat, tgt.lon)

        if dist <= CONTACT_THRESHOLD_M:
            print(f"\n  ★★★ INTERCEPT target #{tgt.id} at {dist:.1f} m "
                  f"(speed: {gs:.1f} m/s) ★★★\n")
            done.append(tgt.id)
            active.remove(tgt)
            guidance = ProNavGuidance()
            continue

        wp = guidance.compute(loc.lat, loc.lon, gs, tgt)
        vehicle.simple_goto(wp)

        brg = bearing_deg(loc.lat, loc.lon, tgt.lat, tgt.lon)
        print(f"  Tgt#{tgt.id}  d={dist:6.1f}m  "
              f"brg={brg:5.1f}°  GS={gs:5.1f}m/s  "
              f"(peak:{peak_gs:.1f})")

        time.sleep(COMMAND_INTERVAL_S)

    print(f"\n══════════════════════════════════════")
    print(f"  MISSION COMPLETE")
    print(f"  Intercepted: {done}")
    print(f"  Peak groundspeed: {peak_gs:.1f} m/s")
    print(f"══════════════════════════════════════\n")

    vehicle.mode = VehicleMode("RTL")
    time.sleep(5)
    vehicle.close()

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    run(p.parse_args().connect)

if __name__ == "__main__":
    main()
