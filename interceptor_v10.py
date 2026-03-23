#!/usr/bin/env python3
"""
Interceptor v10 — Autonomous Multi-Target Drone Intercept (ArduPilot SITL)
==========================================================================

ROOT CAUSE FIXES from v9:
  1. simple_goto() → SET_POSITION_TARGET_LOCAL_NED velocity commands
     simple_goto goes through waypoint planner which caps at ~14 m/s
     regardless of WP_SPD/LOIT_SPEED_MS params. Velocity commands
     bypass the planner entirely — speed limited only by ATC_ANGLE_MAX
     and available thrust.

  2. ProNav killed under 50m → pure pursuit (point directly at target)
     ProNav LOS-rate correction at close range amplifies small bearing
     changes into massive lateral commands → U-turns → orbiting.

  3. Overshoot waypoint removed
     v9 aimed 100m PAST the target. That guarantees a flyby.

  4. Three-phase guidance: PRONAV (>50m) → PURSUIT (15-50m) → RAM (<15m)

  5. Contact threshold raised to 8m (v9 used 10m but oscillated at 15-25m
     because ProNav kept deflecting it away)

  6. Immediate retarget after intercept — no loiter, no pause

  7. Targets sorted nearest-first to minimize drift of later targets

Uses pymavlink directly (not dronekit) for reliable velocity control.
Correct param names for ArduPilot master (2025+).
"""

from __future__ import annotations

import math
import time
import argparse
import signal
import sys
from dataclasses import dataclass, field
from typing import List

from pymavlink import mavutil

# ─── Constants ────────────────────────────────────────────────────

EARTH_RADIUS_M = 6_371_000.0

# Guidance phase thresholds
PRONAV_RANGE_M    = 50.0     # ProNav only when dist > this
RAM_RANGE_M       = 15.0     # Full-send ram when dist < this
CONTACT_M         = 8.0      # Declare intercept

# ProNav tuning (only used > 50m)
PRONAV_N           = 3.5     # Navigation constant
PRONAV_LAT_CLAMP   = 6.0    # Max lateral accel m/s² (prevents wild swings)

# Speed schedule (m/s) — these are COMMANDED velocities via NED,
# NOT limited by WP_SPD or waypoint planner
SPEED_CRUISE       = 25.0    # dist > 200m
SPEED_APPROACH     = 20.0    # 50m < dist < 200m  (ProNav phase)
SPEED_PURSUIT      = 14.0    # 15m < dist < 50m   (pure pursuit)
SPEED_RAM          = 18.0    # dist < 15m          (ram — overshoot OK)

# Altitude
TAKEOFF_ALT_M      = 30.0
INTERCEPT_ALT_M    = 30.0

# Timing
GUIDANCE_DT_S      = 0.05    # 20 Hz guidance loop
BOOT_WAIT_S        = 2.0     # After heartbeat, wait for position stream

# Velocity NED bitmask: ignore position, accel; use velocity + yaw_rate
# Bits: 0=x 1=y 2=z 3=vx 4=vy 5=vz 6=ax 7=ay 8=az 9=force 10=yaw 11=yaw_rate
# We WANT vx,vy,vz,yaw_rate → those bits are 0
# We IGNORE pos(0,1,2), accel(6,7,8), yaw(10) → those bits are 1
VEL_TYPEMASK = 0b0000_1100_0000_0111


# ─── Geometry ─────────────────────────────────────────────────────

def haversine_m(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def destination_point(lat, lon, bearing_d, dist_m):
    rlat = math.radians(lat)
    rlon = math.radians(lon)
    rb = math.radians(bearing_d)
    dr = dist_m / EARTH_RADIUS_M
    new_lat = math.asin(
        math.sin(rlat)*math.cos(dr) + math.cos(rlat)*math.sin(dr)*math.cos(rb))
    new_lon = rlon + math.atan2(
        math.sin(rb)*math.sin(dr)*math.cos(rlat),
        math.cos(dr) - math.sin(rlat)*math.sin(new_lat))
    return math.degrees(new_lat), math.degrees(new_lon)


def wrap_180(a):
    return (a + 180) % 360 - 180


def speed_for_distance(dist):
    if dist > 200:
        return SPEED_CRUISE
    elif dist > PRONAV_RANGE_M:
        return SPEED_APPROACH
    elif dist > RAM_RANGE_M:
        return SPEED_PURSUIT
    else:
        return SPEED_RAM


# ─── Simulated Targets ───────────────────────────────────────────

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
        if dt > 0 and self.speed_mps > 0:
            self.lat, self.lon = destination_point(
                self.lat, self.lon, self.course_deg, self.speed_mps * dt)

    def predicted_pos(self, t_sec):
        return destination_point(
            self.lat, self.lon, self.course_deg, self.speed_mps * t_sec)

    def reset_clock(self):
        self._last_update = time.time()


def build_targets(home_lat, home_lon, alt):
    """Spawn 3 targets near home, sorted nearest-first."""
    targets = [
        SimTarget(id=1, lat=home_lat + 0.0008, lon=home_lon + 0.0005,
                  alt=alt, course_deg=45,  speed_mps=8,  priority=1),
        SimTarget(id=2, lat=home_lat - 0.0005, lon=home_lon + 0.0010,
                  alt=alt, course_deg=120, speed_mps=5,  priority=2),
        SimTarget(id=3, lat=home_lat + 0.0012, lon=home_lon - 0.0003,
                  alt=alt, course_deg=315, speed_mps=10, priority=3),
    ]
    targets.sort(key=lambda t: haversine_m(home_lat, home_lon, t.lat, t.lon))
    return targets


# ─── Hybrid Guidance ──────────────────────────────────────────────

class HybridGuidance:
    """
    Three-phase intercept guidance:
      PRONAV  (dist > 50m)  — proportional navigation with lead prediction
      PURSUIT (15-50m)      — aim directly at target, no lead, no LOS rate
      RAM     (< 15m)       — full speed at target including vertical
    """

    def __init__(self):
        self.prev_los_deg = None
        self.prev_time = None
        self.phase = "INIT"

    def reset(self):
        """Call between targets to clear ProNav state."""
        self.prev_los_deg = None
        self.prev_time = None
        self.phase = "INIT"

    def compute(self, my_lat, my_lon, my_alt, my_vx, my_vy,
                tgt: SimTarget):
        """
        Returns (vn, ve, vd, phase_name).
        vn/ve/vd are NED velocity commands in m/s.
        """
        dist = haversine_m(my_lat, my_lon, tgt.lat, tgt.lon)
        now = time.time()

        # ── Phase selection ──────────────────────────────────────
        if dist > PRONAV_RANGE_M:
            self.phase = "PRONAV"
        elif dist > RAM_RANGE_M:
            self.phase = "PURSUIT"
        else:
            self.phase = "RAM"

        # ── PRONAV: proportional navigation with predicted intercept ──
        if self.phase == "PRONAV":
            my_speed = math.sqrt(my_vx**2 + my_vy**2)
            closing = max(my_speed, 1.0) + tgt.speed_mps
            t_intercept = dist / closing

            # Predict where target will be at intercept time
            pred_lat, pred_lon = tgt.predicted_pos(t_intercept)
            los_deg = bearing_deg(my_lat, my_lon, pred_lat, pred_lon)

            speed = speed_for_distance(dist)
            los_rad = math.radians(los_deg)

            # Base velocity toward predicted intercept point
            vn = speed * math.cos(los_rad)
            ve = speed * math.sin(los_rad)

            # ProNav LOS-rate correction
            if self.prev_los_deg is not None and self.prev_time is not None:
                dt = now - self.prev_time
                if dt > 0.001:
                    los_rate = wrap_180(los_deg - self.prev_los_deg) / dt
                    a_lat = PRONAV_N * closing * math.radians(los_rate)
                    # CLAMP — this is what prevents v9's wild overcorrection
                    a_lat = max(min(a_lat, PRONAV_LAT_CLAMP), -PRONAV_LAT_CLAMP)

                    perp_rad = los_rad + math.pi / 2
                    vn += a_lat * math.cos(perp_rad) * dt
                    ve += a_lat * math.sin(perp_rad) * dt

            self.prev_los_deg = los_deg
            self.prev_time = now

            # Altitude: gentle correction toward target altitude
            alt_err = tgt.alt - my_alt
            vd = -max(min(alt_err * 1.0, 3.0), -3.0)

            return vn, ve, vd, self.phase

        # ── PURSUIT: direct aim at target, NO lead, NO LOS rate ──
        # This is the KEY FIX for oscillation. Under 50m, ProNav's
        # LOS-rate correction amplifies small bearing changes into
        # huge lateral commands, causing U-turns. Pure pursuit just
        # points at the target and goes.
        elif self.phase == "PURSUIT":
            # Clear ProNav state so stale LOS doesn't carry over
            self.prev_los_deg = None
            self.prev_time = None

            brg = bearing_deg(my_lat, my_lon, tgt.lat, tgt.lon)
            speed = speed_for_distance(dist)
            brg_rad = math.radians(brg)

            vn = speed * math.cos(brg_rad)
            ve = speed * math.sin(brg_rad)

            alt_err = tgt.alt - my_alt
            vd = -max(min(alt_err * 1.5, 4.0), -4.0)

            return vn, ve, vd, self.phase

        # ── RAM: full send directly at target ──
        else:
            self.prev_los_deg = None
            self.prev_time = None

            brg = bearing_deg(my_lat, my_lon, tgt.lat, tgt.lon)
            brg_rad = math.radians(brg)

            vn = SPEED_RAM * math.cos(brg_rad)
            ve = SPEED_RAM * math.sin(brg_rad)

            alt_err = tgt.alt - my_alt
            vd = -max(min(alt_err * 2.0, 5.0), -5.0)

            return vn, ve, vd, self.phase


# ─── MAVLink Drone Controller ────────────────────────────────────

class Drone:
    def __init__(self, conn_str):
        print(f"[CONN] Connecting to {conn_str} ...")
        self.mav = mavutil.mavlink_connection(conn_str)
        self.mav.wait_heartbeat()
        self.sys_id = self.mav.target_system
        self.comp_id = self.mav.target_component
        self.boot_time = time.time()
        print(f"[CONN] Heartbeat — system {self.sys_id}")

        # Request ALL data streams at 10 Hz — SITL won't send
        # GLOBAL_POSITION_INT until we ask for it
        print(f"[CONN] Requesting data streams ...")
        for stream_id in range(13):
            self.mav.mav.request_data_stream_send(
                self.sys_id, self.comp_id,
                stream_id, 10, 1)
        time.sleep(1)

        # Wait for GPS fix with progress
        print(f"[CONN] Waiting for GPS fix ...")
        for i in range(120):
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                       blocking=True, timeout=1)
            if msg and msg.lat != 0:
                print(f"[CONN] GPS OK: {msg.lat/1e7:.6f}, {msg.lon/1e7:.6f}")
                break
            if i % 10 == 0 and i > 0:
                print(f"[CONN] Still waiting for GPS ... ({i}s)")
                # Re-request streams in case they didn't stick
                for sid in range(13):
                    self.mav.mav.request_data_stream_send(
                        self.sys_id, self.comp_id, sid, 10, 1)
        else:
            print("[CONN] WARNING: No GPS after 120s, proceeding anyway")

    def _tboot_ms(self):
        return int((time.time() - self.boot_time) * 1000)

    def get_pos(self):
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                   blocking=True, timeout=1)
        if not msg:
            return None
        return {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.relative_alt / 1000.0,
            "vx": msg.vx / 100.0,   # NED m/s
            "vy": msg.vy / 100.0,
            "vz": msg.vz / 100.0,
        }

    def set_mode(self, mode_name):
        mode_map = {
            "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3,
            "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9,
        }
        mode_id = mode_map.get(mode_name, 4)
        self.mav.set_mode_apm(mode_id)
        for _ in range(30):
            hb = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb and hb.custom_mode == mode_id:
                return True
            self.mav.set_mode_apm(mode_id)
            time.sleep(0.3)
        return False

    def arm_and_takeoff(self, alt_m):
        print(f"[MODE] GUIDED ...")
        self.set_mode("GUIDED")

        print(f"[ARM] Arming ...")
        self.mav.arducopter_arm()
        self.mav.motors_armed_wait()
        print(f"[ARM] Armed!")

        print(f"[TAKEOFF] Climbing to {alt_m}m ...")
        self.mav.mav.command_long_send(
            self.sys_id, self.comp_id,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt_m)

        while True:
            pos = self.get_pos()
            if pos and pos["alt"] >= alt_m * 0.90:
                print(f"[TAKEOFF] {pos['alt']:.1f}m ✓")
                return
            time.sleep(0.5)

    def send_vel(self, vn, ve, vd, yaw_rate=0.0):
        """
        THE KEY FIX: Send velocity directly via SET_POSITION_TARGET_LOCAL_NED.

        This bypasses the waypoint planner entirely. The drone tilts to
        achieve the commanded velocity — limited only by ATC_ANGLE_MAX
        (70° ≈ up to ~27 m/s sustainable) and motor thrust.

        v9's simple_goto() → WP planner → capped at ~14 m/s
        This → attitude controller → limited by physics only
        """
        self.mav.mav.set_position_target_local_ned_send(
            self._tboot_ms(),
            self.sys_id, self.comp_id,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            VEL_TYPEMASK,
            0, 0, 0,           # pos (ignored by mask)
            vn, ve, vd,        # velocity NED m/s
            0, 0, 0,           # accel (ignored by mask)
            0, yaw_rate)       # yaw, yaw_rate

    def land(self):
        print("[MODE] RTL ...")
        self.set_mode("RTL")


# ─── Telemetry Logger ─────────────────────────────────────────────

class Logger:
    def __init__(self, path="intercept_v10_log.csv"):
        self.f = open(path, "w")
        self.f.write("t,tgt,phase,dist,speed,vn,ve,vd,"
                     "my_lat,my_lon,my_alt,tgt_lat,tgt_lon\n")

    def log(self, tgt_id, phase, dist, speed, vn, ve, vd,
            my_lat, my_lon, my_alt, tgt_lat, tgt_lon):
        self.f.write(f"{time.time():.3f},{tgt_id},{phase},"
                     f"{dist:.1f},{speed:.1f},{vn:.2f},{ve:.2f},{vd:.2f},"
                     f"{my_lat:.7f},{my_lon:.7f},{my_alt:.1f},"
                     f"{tgt_lat:.7f},{tgt_lon:.7f}\n")

    def close(self):
        self.f.close()


# ─── Force Params via MAVLink ─────────────────────────────────────

def force_params(mav_conn):
    """Belt-and-suspenders: force-set params even if .parm file loaded."""
    params = {
        "ATC_ANGLE_MAX":  70.0,
        "ATC_ANGLE_BOOST": 0.0,
        "WP_SPD":         40.0,
        "LOIT_SPEED_MS":  40.0,
        "LOIT_ACC_MAX_M": 15.0,
        "MOT_THST_HOVER": 0.125,
        "PSC_NE_VEL_P":   6.0,
        "PSC_NE_VEL_I":   2.0,
        "PSC_NE_POS_P":   2.0,
        "PSC_JERK_NE":    20.0,
    }
    print("\n  Forcing params via MAVLink:")
    for name, val in params.items():
        mav_conn.mav.param_set_send(
            mav_conn.target_system, mav_conn.target_component,
            name.encode('utf-8'), val,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.15)
        print(f"    {name} = {val}")
    time.sleep(1)
    print(f"\n  ✓ Velocity commands bypass WP planner (no ~14 m/s cap)")
    print(f"  ✓ ATC_ANGLE_MAX=70° → ~27 m/s theoretical max")
    print(f"  ✓ Freestyle JSON model needed for 30-40 m/s\n")


# ─── Main Mission ─────────────────────────────────────────────────

def run_mission(conn_str):
    drone = Drone(conn_str)
    force_params(drone.mav)

    # Clean exit on Ctrl+C
    def sigint(sig, frame):
        print("\n[SIGINT] Landing ...")
        drone.land()
        time.sleep(3)
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint)

    drone.arm_and_takeoff(TAKEOFF_ALT_M)
    time.sleep(2)

    # Home position → build targets
    pos = drone.get_pos()
    if not pos:
        print("[ERROR] No position!")
        return
    home_lat, home_lon = pos["lat"], pos["lon"]

    targets = build_targets(home_lat, home_lon, INTERCEPT_ALT_M)
    for t in targets:
        t.reset_clock()
        d = haversine_m(home_lat, home_lon, t.lat, t.lon)
        print(f"  Target #{t.id}: {d:.0f}m, spd={t.speed_mps}m/s, "
              f"hdg={t.course_deg}°")

    guidance = HybridGuidance()
    logger = Logger()
    active = list(targets)
    done = []
    peak_gs = 0.0

    print()
    print("═" * 58)
    print("  INTERCEPT v10 — HYBRID GUIDANCE + VELOCITY NED")
    print("  PRONAV (>50m) → PURSUIT (15-50m) → RAM (<15m)")
    print("  Speed: velocity cmds bypass waypoint planner")
    print("═" * 58)
    print()

    mission_t0 = time.time()

    while active:
        pos = drone.get_pos()
        if not pos:
            time.sleep(0.01)
            continue

        my_lat = pos["lat"]
        my_lon = pos["lon"]
        my_alt = pos["alt"]
        my_vx  = pos["vx"]
        my_vy  = pos["vy"]
        my_gs  = math.sqrt(my_vx**2 + my_vy**2)
        peak_gs = max(peak_gs, my_gs)

        # Update all targets (even ones not being chased — they keep moving)
        for t in active:
            t.step()

        # Pick nearest target
        tgt = min(active,
                  key=lambda t: haversine_m(my_lat, my_lon, t.lat, t.lon))
        dist = haversine_m(my_lat, my_lon, tgt.lat, tgt.lon)

        # ── Check intercept ──
        if dist <= CONTACT_M:
            elapsed = time.time() - mission_t0
            print(f"\n  ★★★ INTERCEPT #{tgt.id}  d={dist:.1f}m  "
                  f"GS={my_gs:.1f}m/s  t={elapsed:.1f}s ★★★\n")
            done.append(tgt.id)
            active.remove(tgt)
            guidance.reset()
            continue

        # ── Guidance compute ──
        vn, ve, vd, phase = guidance.compute(
            my_lat, my_lon, my_alt, my_vx, my_vy, tgt)

        # Yaw toward target (cosmetic, not required)
        brg = bearing_deg(my_lat, my_lon, tgt.lat, tgt.lon)
        heading = (math.degrees(math.atan2(my_vy, my_vx)) + 360) % 360
        yaw_err = wrap_180(brg - heading)
        yaw_rate = max(min(yaw_err * 0.05, 1.0), -1.0)

        # ── Send velocity command ──
        drone.send_vel(vn, ve, vd, yaw_rate)

        # ── Log ──
        cmd_spd = math.sqrt(vn**2 + ve**2)
        logger.log(tgt.id, phase, dist, my_gs, vn, ve, vd,
                   my_lat, my_lon, my_alt, tgt.lat, tgt.lon)

        print(f"  T#{tgt.id} [{phase:7s}] "
              f"d={dist:6.1f}m  "
              f"GS={my_gs:5.1f} cmd={cmd_spd:5.1f}  "
              f"pk={peak_gs:.1f}")

        time.sleep(GUIDANCE_DT_S)

    # ── Done ──
    logger.close()
    elapsed = time.time() - mission_t0

    print()
    print("═" * 58)
    print(f"  MISSION COMPLETE — {elapsed:.1f}s total")
    print(f"  Intercepted: {done}")
    print(f"  Peak groundspeed: {peak_gs:.1f} m/s")
    print("═" * 58)
    print()

    drone.land()
    time.sleep(5)


def main():
    p = argparse.ArgumentParser(
        description="Interceptor v10 — Hybrid Guidance Intercept")
    p.add_argument("--connect", default="tcp:127.0.0.1:5760",
                   help="MAVLink connection string")
    run_mission(p.parse_args().connect)


if __name__ == "__main__":
    main()
