#!/usr/bin/env python3
"""
Interceptor v9 — Autonomous UAV Intercept Simulation (ArduPilot SITL)
======================================================================
Drone speed: 40 m/s (144 km/h)

Navigates a simulated drone to intercept a moving virtual target using
overshoot-based simple_goto commands (bypasses the SET_POSITION_TARGET
velocity bug where groundspeed stays at 0 m/s).

Key techniques:
  • Overshoot navigation  — aims 50 m past the target so the flight
    controller never decelerates before reaching the waypoint coords.
  • Proportional Navigation (ProNav) — bearing is biased toward the
    predicted future position of the target.
  • Priority-based waypoint selection — when multiple targets exist the
    closest / highest-priority one is serviced first.
  • Runtime param override — forces WPNAV_SPEED to 4000 (40 m/s) via
    MAVLink after connecting.

Requires:  dronekit   (pip install dronekit)
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
DRONE_SPEED_CMS: float = 4000.0          # 40 m/s in cm/s
OVERSHOOT_DISTANCE_M: float = 50.0       # aim this far past the target
CONTACT_THRESHOLD_M: float = 10.0        # "intercept" when closer than this
COMMAND_INTERVAL_S: float = 0.25          # guidance-loop period
DEFAULT_ALT_M: float = 30.0              # cruise altitude AGL
EARTH_RADIUS_M: float = 6_371_000.0
NAV_GAIN: float = 3.0                    # ProNav proportionality constant (N)

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two WGS-84 points."""
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = (math.sin(dlat / 2) ** 2
         + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing (degrees, 0=N, clockwise) from point 1 → point 2."""
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = (math.cos(rlat1) * math.sin(rlat2)
         - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def destination_point(lat: float, lon: float,
                      bearing_d: float, dist_m: float) -> tuple[float, float]:
    """Return (lat, lon) after travelling *dist_m* along *bearing_d*."""
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
    """A virtual target that moves along a straight course."""
    id: int
    lat: float
    lon: float
    alt: float
    course_deg: float        # ground track (degrees)
    speed_mps: float         # ground speed (m/s)
    priority: int = 1        # lower = higher priority
    _last_update: float = field(default_factory=time.time, repr=False)

    def step(self) -> None:
        """Advance the target position by elapsed wall-clock time."""
        now = time.time()
        dt = now - self._last_update
        self._last_update = now
        dist = self.speed_mps * dt
        self.lat, self.lon = destination_point(
            self.lat, self.lon, self.course_deg, dist,
        )

    def predicted_position(self, t_sec: float) -> tuple[float, float]:
        """Where the target will be *t_sec* from now (constant-velocity)."""
        dist = self.speed_mps * t_sec
        return destination_point(self.lat, self.lon, self.course_deg, dist)

# ---------------------------------------------------------------------------
# Priority-based target selector
# ---------------------------------------------------------------------------

def select_target(
    targets: List[SimTarget],
    drone_lat: float,
    drone_lon: float,
) -> Optional[SimTarget]:
    """Pick the best target: lowest priority number first, then nearest."""
    if not targets:
        return None
    return min(
        targets,
        key=lambda t: (
            t.priority,
            haversine_m(drone_lat, drone_lon, t.lat, t.lon),
        ),
    )

# ---------------------------------------------------------------------------
# ProNav + Overshoot guidance
# ---------------------------------------------------------------------------

class ProNavGuidance:
    """Proportional-Navigation guidance that outputs an overshoot waypoint."""

    def __init__(self, gain: float = NAV_GAIN,
                 overshoot_m: float = OVERSHOOT_DISTANCE_M):
        self.gain = gain
        self.overshoot_m = overshoot_m
        self._prev_los_deg: Optional[float] = None
        self._prev_time: Optional[float] = None

    def compute(
        self,
        drone_lat: float, drone_lon: float, drone_speed: float,
        target: SimTarget,
    ) -> LocationGlobalRelative:
        """
        Return a LocationGlobalRelative that is *overshoot_m* past the
        predicted intercept point so simple_goto never decelerates early.
        """
        now = time.time()
        dist = haversine_m(drone_lat, drone_lon, target.lat, target.lon)

        # Time-to-intercept estimate (avoid /0)
        closing_speed = max(drone_speed, 1.0) + target.speed_mps
        t_intercept = dist / closing_speed

        # Predicted target position at intercept time
        pred_lat, pred_lon = target.predicted_position(t_intercept)

        # Line-of-sight bearing and its rate
        los_deg = bearing_deg(drone_lat, drone_lon, pred_lat, pred_lon)

        if self._prev_los_deg is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                d_los = (los_deg - self._prev_los_deg + 180) % 360 - 180
                los_rate = d_los / dt  # deg/s
                # ProNav correction: steer N × LOS-rate ahead
                los_deg = (los_deg + self.gain * los_rate * dt) % 360

        self._prev_los_deg = los_deg
        self._prev_time = now

        # Project overshoot point past predicted position
        aim_lat, aim_lon = destination_point(
            pred_lat, pred_lon, los_deg, self.overshoot_m,
        )
        return LocationGlobalRelative(aim_lat, aim_lon, target.alt)

# ---------------------------------------------------------------------------
# Vehicle helpers
# ---------------------------------------------------------------------------

def set_param_safe(vehicle, name: str, value: float) -> None:
    """Set a parameter via MAVLink using raw PARAM_SET message.
    This avoids dronekit's buggy readback timeout."""
    msg = vehicle.message_factory.param_set_encode(
        0, 0,                                    # target system, component
        name.encode("utf-8"),                     # param_id (bytes)
        value,                                    # param_value
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,    # param_type
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def force_speed_params(vehicle) -> None:
    """Override speed params at runtime via MAVLink so no reboot is needed."""
    params = {
        "WPNAV_SPEED":  DRONE_SPEED_CMS,   # 4000 = 40 m/s
        "WPNAV_ACCEL":  800,                # 8 m/s²
        "LOIT_SPEED":   DRONE_SPEED_CMS,    # match
        "LOIT_ACC_MAX": 800,
        "ANGLE_MAX":    4500,               # 45° lean for high speed
        "PSC_VELXY_P":  5.0,                # aggressive velocity tracking
    }
    print("  Forcing speed parameters via MAVLink:")
    for name, value in params.items():
        set_param_safe(vehicle, name, value)
        print(f"    {name} = {value}")
        time.sleep(0.3)

    # Wait for params to take effect
    time.sleep(3)
    print("  Parameters sent. Drone target speed: 40 m/s")


def wait_for_armable(vehicle, timeout: float = 120.0) -> None:
    print("  Waiting for vehicle to become armable …")
    t0 = time.time()
    while not vehicle.is_armable:
        if time.time() - t0 > timeout:
            raise TimeoutError("Vehicle never became armable")
        time.sleep(0.5)
    print("  Vehicle is armable.")


def arm_and_takeoff(vehicle, alt_m: float) -> None:
    """Switch to GUIDED, arm, take off, and wait until target alt reached."""
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
# Main simulation loop
# ---------------------------------------------------------------------------

def run_simulation(connection_string: str, targets: List[SimTarget]) -> None:
    print(f"Connecting to vehicle on {connection_string} …")
    vehicle = connect(connection_string, wait_ready=True)

    # Force speed params BEFORE arming
    force_speed_params(vehicle)

    # Graceful Ctrl-C
    def _sig(sig, frame):
        print("\n[SIGINT] Landing …")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(2)
        vehicle.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sig)

    arm_and_takeoff(vehicle, DEFAULT_ALT_M)

    guidance = ProNavGuidance()
    active_targets = list(targets)
    intercepted: list[int] = []

    print("\n=== Intercept loop started ===\n")
    print(f"  Drone max speed: {DRONE_SPEED_CMS/100:.0f} m/s")
    print(f"  Overshoot: {OVERSHOOT_DISTANCE_M:.0f} m")
    print(f"  Contact threshold: {CONTACT_THRESHOLD_M:.0f} m")
    print(f"  Targets: {len(active_targets)}\n")

    while active_targets:
        loc = vehicle.location.global_relative_frame
        d_lat, d_lon = loc.lat, loc.lon
        groundspeed = vehicle.groundspeed or 0.0

        # Advance all target positions
        for t in active_targets:
            t.step()

        # Pick best target
        tgt = select_target(active_targets, d_lat, d_lon)
        if tgt is None:
            break

        dist = haversine_m(d_lat, d_lon, tgt.lat, tgt.lon)

        # Check intercept
        if dist <= CONTACT_THRESHOLD_M:
            print(f"\n  ★ INTERCEPT target #{tgt.id} at {dist:.1f} m  "
                  f"(gs={groundspeed:.1f} m/s)\n")
            intercepted.append(tgt.id)
            active_targets.remove(tgt)
            guidance = ProNavGuidance()          # reset LOS state
            continue

        # Compute overshoot waypoint and command vehicle
        wp = guidance.compute(d_lat, d_lon, groundspeed, tgt)
        vehicle.simple_goto(wp)

        brg = bearing_deg(d_lat, d_lon, tgt.lat, tgt.lon)
        print(
            f"  Tgt #{tgt.id}  dist={dist:6.1f} m  "
            f"brg={brg:5.1f}°  gs={groundspeed:5.1f} m/s  "
            f"aim→({wp.lat:.6f}, {wp.lon:.6f})"
        )

        time.sleep(COMMAND_INTERVAL_S)

    # Done
    print(f"\nAll targets intercepted: {intercepted}")
    print("Returning to launch …")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(5)
    vehicle.close()

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def build_default_targets() -> List[SimTarget]:
    """Three moving targets near the SITL default home (Canberra).
    Target speeds: 5-12 m/s (drone does 40 m/s, so these are catchable).
    """
    home_lat, home_lon = -35.363262, 149.165237
    return [
        SimTarget(
            id=1, priority=1,
            lat=home_lat + 0.002, lon=home_lon + 0.001,
            alt=DEFAULT_ALT_M,
            course_deg=90, speed_mps=8,
        ),
        SimTarget(
            id=2, priority=2,
            lat=home_lat - 0.001, lon=home_lon + 0.003,
            alt=DEFAULT_ALT_M,
            course_deg=180, speed_mps=5,
        ),
        SimTarget(
            id=3, priority=3,
            lat=home_lat + 0.003, lon=home_lon - 0.002,
            alt=DEFAULT_ALT_M,
            course_deg=270, speed_mps=12,
        ),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="UAV intercept simulation (ArduPilot SITL)",
    )
    parser.add_argument(
        "--connect", default="tcp:127.0.0.1:5760",
        help="MAVLink connection string (default: tcp:127.0.0.1:5760)",
    )
    args = parser.parse_args()
    targets = build_default_targets()
    run_simulation(args.connect, targets)


if __name__ == "__main__":
    main()
