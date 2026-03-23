#!/usr/bin/env python3
"""
Interceptor v10 — Built on v9's proven dronekit boot sequence
=============================================================
KEPT FROM v9 (fast boot, works):
  - dronekit connect/arm/takeoff — UNCHANGED
  - same target spawning
  - same param forcing

CHANGED (fixes oscillation + speed):
  1. simple_goto → velocity NED commands (bypasses 14 m/s cap)
  2. ProNav killed under 50m → pure pursuit (fixes oscillation)
  3. Ram mode under 15m
  4. Overshoot waypoint REMOVED
  5. Targets sorted nearest-first
  6. Contact threshold 8m
"""

from __future__ import annotations

import math
import time
import argparse
import signal
import sys
from dataclasses import dataclass, field

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CONTACT_THRESHOLD_M = 8.0
COMMAND_INTERVAL_S  = 0.05
DEFAULT_ALT_M       = 30.0
EARTH_RADIUS_M      = 6_371_000.0

PRONAV_RANGE_M      = 50.0
RAM_RANGE_M         = 15.0
PRONAV_N            = 3.5
PRONAV_CLAMP        = 6.0

SPEED_CRUISE        = 25.0
SPEED_APPROACH      = 20.0
SPEED_PURSUIT       = 14.0
SPEED_RAM           = 18.0

VEL_TYPEMASK = 0b0000_1100_0000_0111

# ---------------------------------------------------------------------------
# Geometry (same as v9)
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

def wrap_180(a):
    return (a + 180) % 360 - 180

def speed_for_dist(d):
    if d > 200: return SPEED_CRUISE
    if d > PRONAV_RANGE_M: return SPEED_APPROACH
    if d > RAM_RANGE_M: return SPEED_PURSUIT
    return SPEED_RAM

# ---------------------------------------------------------------------------
# Target (same as v9)
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
# v9 vehicle helpers (UNCHANGED)
# ---------------------------------------------------------------------------

def set_param(vehicle, name, value):
    msg = vehicle.message_factory.param_set_encode(
        0, 0, name.encode("utf-8"), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def force_speed_params(vehicle):
    params = {
        "ATC_ANGLE_MAX":  70.0,
        "ATC_ANGLE_BOOST": 0.0,
        "ATC_ACC_P_MAX":  2200.0,
        "ATC_ACC_R_MAX":  2200.0,
        "WP_SPD":         40.0,
        "LOIT_SPEED_MS":  40.0,
        "LOIT_ACC_MAX_M": 15.0,
        "LOIT_BRK_ACC_M": 8.0,
        "LOIT_BRK_DELAY": 0.1,
        "LOIT_BRK_JRK_M": 15.0,
        "MOT_THST_HOVER": 0.125,
        "PSC_NE_VEL_P":   6.0,
        "PSC_NE_VEL_I":   2.0,
        "PSC_NE_POS_P":   2.0,
        "PSC_JERK_NE":    20.0,
    }
    print("\n  Forcing params:")
    for k, v in params.items():
        set_param(vehicle, k, v)
        print(f"    {k} = {v}")
        time.sleep(0.2)
    time.sleep(3)
    print(f"\n  v10: velocity NED (no 14 m/s cap)")
    print(f"  v10: PRONAV > PURSUIT > RAM\n")

def wait_armable(vehicle, timeout=120):
    print("  Waiting for armable ...")
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
    print("  Armed! Taking off ...")
    vehicle.simple_takeoff(alt)
    while True:
        cur = vehicle.location.global_relative_frame.alt or 0
        if cur >= alt * 0.95:
            print(f"  Alt {cur:.1f} m -- takeoff done.\n")
            break
        time.sleep(0.5)

# ---------------------------------------------------------------------------
# NEW: Velocity NED command (replaces simple_goto)
# ---------------------------------------------------------------------------

def send_velocity_ned(vehicle, vn, ve, vd, yaw_rate=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VEL_TYPEMASK,
        0, 0, 0,
        vn, ve, vd,
        0, 0, 0,
        0, yaw_rate)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ---------------------------------------------------------------------------
# NEW: Hybrid Guidance (replaces ProNav-only)
# ---------------------------------------------------------------------------

class HybridGuidance:
    def __init__(self):
        self._prev_los = None
        self._prev_time = None
        self.phase = "INIT"

    def reset(self):
        self._prev_los = None
        self._prev_time = None
        self.phase = "INIT"

    def compute(self, drone_lat, drone_lon, drone_alt, drone_speed, target):
        dist = haversine_m(drone_lat, drone_lon, target.lat, target.lon)
        now = time.time()

        if dist > PRONAV_RANGE_M:
            self.phase = "PRONAV"
            closing = max(drone_speed, 1.0) + target.speed_mps
            t_int = dist / closing
            pred_lat, pred_lon = target.predicted_position(t_int)
            los = bearing_deg(drone_lat, drone_lon, pred_lat, pred_lon)
            speed = speed_for_dist(dist)
            los_rad = math.radians(los)
            vn = speed * math.cos(los_rad)
            ve = speed * math.sin(los_rad)
            if self._prev_los is not None and self._prev_time is not None:
                dt = now - self._prev_time
                if dt > 0.001:
                    rate = wrap_180(los - self._prev_los) / dt
                    a_lat = PRONAV_N * closing * math.radians(rate)
                    a_lat = max(min(a_lat, PRONAV_CLAMP), -PRONAV_CLAMP)
                    perp = los_rad + math.pi / 2
                    vn += a_lat * math.cos(perp) * dt
                    ve += a_lat * math.sin(perp) * dt
            self._prev_los = los
            self._prev_time = now
            alt_err = target.alt - drone_alt
            vd = -max(min(alt_err * 1.0, 3.0), -3.0)
            return vn, ve, vd

        elif dist > RAM_RANGE_M:
            self.phase = "PURSUIT"
            self._prev_los = None
            self._prev_time = None
            b = bearing_deg(drone_lat, drone_lon, target.lat, target.lon)
            speed = speed_for_dist(dist)
            br = math.radians(b)
            vn = speed * math.cos(br)
            ve = speed * math.sin(br)
            alt_err = target.alt - drone_alt
            vd = -max(min(alt_err * 1.5, 4.0), -4.0)
            return vn, ve, vd

        else:
            self.phase = "RAM"
            self._prev_los = None
            self._prev_time = None
            b = bearing_deg(drone_lat, drone_lon, target.lat, target.lon)
            br = math.radians(b)
            vn = SPEED_RAM * math.cos(br)
            ve = SPEED_RAM * math.sin(br)
            alt_err = target.alt - drone_alt
            vd = -max(min(alt_err * 2.0, 5.0), -5.0)
            return vn, ve, vd

# ---------------------------------------------------------------------------
# Build targets — sorted nearest first
# ---------------------------------------------------------------------------

def build_targets(lat, lon):
    targets = [
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
    targets.sort(key=lambda t: haversine_m(lat, lon, t.lat, t.lon))
    return targets

# ---------------------------------------------------------------------------
# Main — v9 boot + v10 intercept
# ---------------------------------------------------------------------------

def run(conn):
    print(f"Connecting to {conn} ...")
    vehicle = connect(conn, wait_ready=True, heartbeat_timeout=60)
    force_speed_params(vehicle)

    def _sig(s, f):
        print("\n[SIGINT] Landing ...")
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
              f"hdg {t.course_deg}")

    guidance = HybridGuidance()
    active = list(targets)
    done = []
    peak_gs = 0.0
    t0 = time.time()

    print("\n" + "=" * 50)
    print("  INTERCEPT v10 — HYBRID GUIDANCE")
    print("  PRONAV > PURSUIT > RAM")
    print("  Velocity NED — no speed cap")
    print("=" * 50 + "\n")

    while active:
        loc = vehicle.location.global_relative_frame
        gs = vehicle.groundspeed or 0.0
        alt = loc.alt or 0.0
        peak_gs = max(peak_gs, gs)

        for t in active:
            t.step()

        tgt = min(active, key=lambda t: haversine_m(
            loc.lat, loc.lon, t.lat, t.lon))
        dist = haversine_m(loc.lat, loc.lon, tgt.lat, tgt.lon)

        if dist <= CONTACT_THRESHOLD_M:
            elapsed = time.time() - t0
            print(f"\n  *** INTERCEPT #{tgt.id} d={dist:.1f}m "
                  f"GS={gs:.1f}m/s t={elapsed:.1f}s ***\n")
            done.append(tgt.id)
            active.remove(tgt)
            guidance.reset()
            continue

        vn, ve, vd = guidance.compute(loc.lat, loc.lon, alt, gs, tgt)
        send_velocity_ned(vehicle, vn, ve, vd)

        cmd_spd = math.sqrt(vn**2 + ve**2)
        print(f"  T#{tgt.id} [{guidance.phase:7s}] "
              f"d={dist:6.1f}m  GS={gs:5.1f}  "
              f"cmd={cmd_spd:5.1f}  pk={peak_gs:.1f}")

        time.sleep(COMMAND_INTERVAL_S)

    elapsed = time.time() - t0
    print(f"\n" + "=" * 50)
    print(f"  MISSION COMPLETE -- {elapsed:.1f}s")
    print(f"  Intercepted: {done}")
    print(f"  Peak groundspeed: {peak_gs:.1f} m/s")
    print("=" * 50 + "\n")

    vehicle.mode = VehicleMode("RTL")
    time.sleep(5)
    vehicle.close()

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    run(p.parse_args().connect)

if __name__ == "__main__":
    main()
