#!/usr/bin/env python3
"""
Interceptor v9 — Autonomous UAV Intercept Simulation (ArduPilot SITL)
======================================================================
Navigates a simulated drone to intercept moving virtual targets using
overshoot-based simple_goto commands.

The default SITL quadcopter frame maxes out around 17-20 m/s regardless
of WPNAV_SPEED setting (motor/prop physics limit). This script is
designed to work within that real limit.

Targets spawn AFTER takeoff so they don't drift away during boot.
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
OVERSHOOT_DISTANCE_M: float = 80.0       # aim this far past the target
CONTACT_THRESHOLD_M: float = 10.0        # "intercept" when closer than this
COMMAND_INTERVAL_S: float = 0.20          # guidance-loop period (5 Hz)
DEFAULT_ALT_M: float = 30.0              # cruise altitude AGL
EARTH_RADIUS_M: float = 6_371_000.0
NAV_GAIN: float = 3.0                    # ProNav proportionality constant

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = (math.sin(dlat / 2) ** 2
         + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = (math.cos(rlat1) * math.sin(rlat2)
         - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def destination_point(lat: float, lon: float,
                      bearing_d: float, dist_m: float) -> tuple[float, float]:
    rlat = math.radians(lat)
    rlon = math.radians(lon)
    rb = math.radians(bearing_d)
    d_r = dist_m / EARTH_RADIUS_M
    new_lat = math.asin(
        math.sin(rlat) * math.cos(d_r)
        + math.cos(rlat) * math.sin(d_r) * math.cos(rb)
    )
    new_lon = rlon + math.atan2(
        math.sin(rb) * math.sin(d_r) * math.cos(rlat),
        math.cos(d_r) - math.sin(rlat) * math.sin(new_lat),
    )
    return math.degrees(new_lat), math.degrees(new_lon)

# ---------------------------------------------------------------------------
# Target (simulated moving waypoint)
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

    def step(self) -> None:
        now = time.time()
        dt = now - self._last_update
        self._last_update = now
        dist = self.speed_mps * dt
        self.lat, self.lon = destination_point(
            self.lat, self.lon, self.course_deg, dist)

    def predicted_position(self, t_sec: float) -> tuple[float, float]:
        dist = self.speed_mps * t_sec
        return destination_point(self.lat, self.lon, self.course_deg, dist)

    def reset_clock(self) -> None:
        """Reset the internal clock — call right before the chase starts."""
        self._last_update = time.time()

# ---------------------------------------------------------------------------
# Priority-based target selector
# ---------------------------------------------------------------------------

def select_target(targets: List[SimTarget],
                  drone_lat: float, drone_lon: float) -> Optional[SimTarget]:
    if not targets:
        return None
    return min(targets, key=lambda t: (
        t.priority,
        haversine_m(drone_lat, drone_lon, t.lat, t.lon)))

# ---------------------------------------------------------------------------
# ProNav + Overshoot guidance
# ---------------------------------------------------------------------------

class ProNavGuidance:
    def __init__(self, gain: float = NAV_GAIN,
                 overshoot_m: float = OVERSHOOT_DISTANCE_M):
        self.gain = gain
        self.overshoot_m = overshoot_m
        self._prev_los_deg: Optional[float] = None
        self._prev_time: Optional[float] = None

    def compute(self, drone_lat: float, drone_lon: float,
                drone_speed: float, target: SimTarget) -> LocationGlobalRelative:
        now = time.time()
        dist = haversine_m(drone_lat, drone_lon, target.lat, target.lon)

        closing_speed = max(drone_speed, 1.0) + target.speed_mps
        t_intercept = dist / closing_speed

        pred_lat, pred_lon = target.predicted_position(t_intercept)
        los_deg = bearing_deg(drone_lat, drone_lon, pred_lat, pred_lon)

        if self._prev_los_deg is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                d_los = (los_deg - self._prev_los_deg + 180) % 360 - 180
                los_rate = d_los / dt
                los_deg = (los_deg + self.gain * los_rate * dt) % 360

        self._prev_los_deg = los_deg
        self._prev_time = now

        aim_lat, aim_lon = destination_point(
            pred_lat, pred_lon, los_deg, self.overshoot_m)
        return LocationGlobalRelative(aim_lat, aim_lon, target.alt)

# ---------------------------------------------------------------------------
# Vehicle helpers
# ---------------------------------------------------------------------------

def set_param_safe(vehicle, name: str, value: float) -> None:
    msg = vehicle.message_factory.param_set_encode(
        0, 0, name.encode("utf-8"), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def force_speed_params(vehicle) -> None:
    params = {
        "WPNAV_SPEED":  4000,
        "WPNAV_ACCEL":  800,
        "LOIT_SPEED":   4000,
        "LOIT_ACC_MAX": 800,
        "ANGLE_MAX":    4500,
        "PSC_VELXY_P":  5.0,
    }
    print("  Forcing speed parameters via MAVLink:")
    for name, value in params.items():
        set_param_safe(vehicle, name, value)
        print(f"    {name} = {value}")
        time.sleep(0.3)
    time.sleep(3)
    print("  Parameters sent.")


def wait_for_armable(vehicle, timeout: float = 120.0) -> None:
    print("  Waiting for vehicle to become armable …")
    t0 = time.time()
    while not vehicle.is_armable:
        if time.time() - t0 > timeout:
            raise TimeoutError("Vehicle never became armable")
        time.sleep(0.5)
    print("  Vehicle is armable.")


def arm_and_takeoff(vehicle, alt_m: float) -> None:
    wait_for_armable(vehicle)

    print("  Setting GUIDED mode …")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.3)

    print("  Arming motors …")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.3)
    print("  Armed!")

    print(f"  Taking off to {alt_m} m …")
    vehicle.simple_takeoff(alt_m)
    while True:
        current_alt = vehicle.location.global_relative_frame.alt or 0
        if current_alt >= alt_m * 0.95:
            print(f"  Reached {current_alt:.1f} m — takeoff complete.")
            break
        time.sleep(0.5)

# ---------------------------------------------------------------------------
# Build targets relative to CURRENT drone position (after takeoff)
# ---------------------------------------------------------------------------

def build_targets(drone_lat: float, drone_lon: float) -> List[SimTarget]:
    """Spawn targets 80-150 m from the drone's current position.
    Targets move ACROSS the drone's path (not directly away) so
    the drone can close distance even at moderate speeds.
    """
    return [
        SimTarget(
            id=1, priority=1,
            lat=drone_lat + 0.0008,    # ~89 m north
            lon=drone_lon + 0.0005,    # ~44 m east
            alt=DEFAULT_ALT_M,
            course_deg=45,             # moving northeast
            speed_mps=5,               # slow target
        ),
        SimTarget(
            id=2, priority=2,
            lat=drone_lat - 0.0005,    # ~56 m south
            lon=drone_lon + 0.0010,    # ~88 m east
            alt=DEFAULT_ALT_M,
            course_deg=120,            # moving southeast
            speed_mps=3,               # very slow
        ),
        SimTarget(
            id=3, priority=3,
            lat=drone_lat + 0.0012,    # ~133 m north
            lon=drone_lon - 0.0003,    # ~26 m west
            alt=DEFAULT_ALT_M,
            course_deg=315,            # moving northwest
            speed_mps=7,               # faster target
        ),
    ]

# ---------------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------------

def run_simulation(connection_string: str) -> None:
    print(f"Connecting to vehicle on {connection_string} …")
    vehicle = connect(connection_string, wait_ready=True)

    force_speed_params(vehicle)

    def _sig(sig, frame):
        print("\n[SIGINT] Landing …")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(2)
        vehicle.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, _sig)

    arm_and_takeoff(vehicle, DEFAULT_ALT_M)

    # === Spawn targets NOW (after takeoff) so they start close ===
    loc = vehicle.location.global_relative_frame
    targets = build_targets(loc.lat, loc.lon)
    for t in targets:
        t.reset_clock()
        d = haversine_m(loc.lat, loc.lon, t.lat, t.lon)
        print(f"  Target #{t.id}: {d:.0f} m away, speed {t.speed_mps} m/s, "
              f"heading {t.course_deg}°")

    guidance = ProNavGuidance()
    active_targets = list(targets)
    intercepted: list[int] = []

    print("\n=== Intercept loop started ===")
    print(f"  Overshoot: {OVERSHOOT_DISTANCE_M:.0f} m")
    print(f"  Contact threshold: {CONTACT_THRESHOLD_M:.0f} m")
    print(f"  Targets: {len(active_targets)}\n")

    while active_targets:
        loc = vehicle.location.global_relative_frame
        d_lat, d_lon = loc.lat, loc.lon
        groundspeed = vehicle.groundspeed or 0.0

        for t in active_targets:
            t.step()

        tgt = select_target(active_targets, d_lat, d_lon)
        if tgt is None:
            break

        dist = haversine_m(d_lat, d_lon, tgt.lat, tgt.lon)

        if dist <= CONTACT_THRESHOLD_M:
            print(f"\n  ★ INTERCEPT target #{tgt.id} at {dist:.1f} m  "
                  f"(gs={groundspeed:.1f} m/s)\n")
            intercepted.append(tgt.id)
            active_targets.remove(tgt)
            guidance = ProNavGuidance()
            continue

        wp = guidance.compute(d_lat, d_lon, groundspeed, tgt)
        vehicle.simple_goto(wp)

        brg = bearing_deg(d_lat, d_lon, tgt.lat, tgt.lon)
        print(
            f"  Tgt #{tgt.id}  dist={dist:6.1f} m  "
            f"brg={brg:5.1f}°  gs={groundspeed:5.1f} m/s  "
            f"aim→({wp.lat:.6f}, {wp.lon:.6f})"
        )

        time.sleep(COMMAND_INTERVAL_S)

    print(f"\nAll targets intercepted: {intercepted}")
    print("Returning to launch …")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(5)
    vehicle.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="UAV intercept simulation (ArduPilot SITL)")
    parser.add_argument(
        "--connect", default="tcp:127.0.0.1:5760",
        help="MAVLink connection string (default: tcp:127.0.0.1:5760)")
    args = parser.parse_args()
    run_simulation(args.connect)


if __name__ == "__main__":
    main()
