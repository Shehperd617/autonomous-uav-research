#!/usr/bin/env python3
"""
Interceptor v10 — Autonomous Multi-Target Drone Intercept (ArduPilot SITL)
==========================================================================
Hybrid guidance: PRONAV (>50m) → PURSUIT (15-50m) → RAM (<15m)
Velocity NED commands bypass waypoint planner speed cap.
pymavlink only — no dronekit dependency.
"""

import math
import time
import argparse
import signal
import sys

from pymavlink import mavutil

# ─── Constants ────────────────────────────────────────────────────

EARTH_RADIUS_M     = 6_371_000.0
PRONAV_RANGE_M     = 50.0
RAM_RANGE_M        = 15.0
CONTACT_M          = 8.0
PRONAV_N           = 3.5
PRONAV_LAT_CLAMP   = 6.0
SPEED_CRUISE       = 25.0
SPEED_APPROACH     = 20.0
SPEED_PURSUIT      = 14.0
SPEED_RAM          = 18.0
TAKEOFF_ALT_M      = 15.0
INTERCEPT_ALT_M    = 15.0
GUIDANCE_DT_S      = 0.05
VEL_TYPEMASK       = 0b0000_1100_0000_0111

# ─── Geometry ─────────────────────────────────────────────────────

def haversine_m(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def bearing_deg(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360

def destination_point(lat, lon, brg_d, dist_m):
    rlat, rlon, rb = math.radians(lat), math.radians(lon), math.radians(brg_d)
    dr = dist_m / EARTH_RADIUS_M
    nlat = math.asin(math.sin(rlat)*math.cos(dr) + math.cos(rlat)*math.sin(dr)*math.cos(rb))
    nlon = rlon + math.atan2(math.sin(rb)*math.sin(dr)*math.cos(rlat),
                              math.cos(dr) - math.sin(rlat)*math.sin(nlat))
    return math.degrees(nlat), math.degrees(nlon)

def wrap_180(a):
    return (a + 180) % 360 - 180

def speed_for_dist(d):
    if d > 200: return SPEED_CRUISE
    if d > PRONAV_RANGE_M: return SPEED_APPROACH
    if d > RAM_RANGE_M: return SPEED_PURSUIT
    return SPEED_RAM

# ─── Simulated Targets ───────────────────────────────────────────

class Target:
    def __init__(self, tid, lat, lon, alt, course, speed):
        self.id = tid
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.course = course
        self.speed = speed
        self._t = time.time()

    def step(self):
        now = time.time()
        dt = now - self._t
        self._t = now
        if dt > 0 and self.speed > 0:
            self.lat, self.lon = destination_point(self.lat, self.lon, self.course, self.speed * dt)

    def predict(self, t_sec):
        return destination_point(self.lat, self.lon, self.course, self.speed * t_sec)

    def reset_clock(self):
        self._t = time.time()

def build_targets(hlat, hlon, alt):
    targets = [
        Target(1, hlat + 0.0008, hlon + 0.0005, alt, 45,  8),
        Target(2, hlat - 0.0005, hlon + 0.0010, alt, 120, 5),
        Target(3, hlat + 0.0012, hlon - 0.0003, alt, 315, 10),
    ]
    targets.sort(key=lambda t: haversine_m(hlat, hlon, t.lat, t.lon))
    return targets

# ─── Hybrid Guidance ──────────────────────────────────────────────

class Guidance:
    def __init__(self):
        self.prev_los = None
        self.prev_t = None
        self.phase = "INIT"

    def reset(self):
        self.prev_los = None
        self.prev_t = None

    def compute(self, mlat, mlon, malt, mvx, mvy, tgt):
        dist = haversine_m(mlat, mlon, tgt.lat, tgt.lon)
        now = time.time()

        if dist > PRONAV_RANGE_M:
            self.phase = "PRONAV"
            mspd = math.sqrt(mvx**2 + mvy**2)
            closing = max(mspd, 1.0) + tgt.speed
            t_int = dist / closing
            plat, plon = tgt.predict(t_int)
            los = bearing_deg(mlat, mlon, plat, plon)
            spd = speed_for_dist(dist)
            lr = math.radians(los)
            vn = spd * math.cos(lr)
            ve = spd * math.sin(lr)
            if self.prev_los is not None and self.prev_t is not None:
                dt = now - self.prev_t
                if dt > 0.001:
                    rate = wrap_180(los - self.prev_los) / dt
                    alat = PRONAV_N * closing * math.radians(rate)
                    alat = max(min(alat, PRONAV_LAT_CLAMP), -PRONAV_LAT_CLAMP)
                    pr = lr + math.pi / 2
                    vn += alat * math.cos(pr) * dt
                    ve += alat * math.sin(pr) * dt
            self.prev_los = los
            self.prev_t = now
            aerr = tgt.alt - malt
            vd = -max(min(aerr * 1.0, 3.0), -3.0)
            return vn, ve, vd, self.phase

        elif dist > RAM_RANGE_M:
            self.phase = "PURSUIT"
            self.prev_los = None
            self.prev_t = None
            brg = bearing_deg(mlat, mlon, tgt.lat, tgt.lon)
            spd = speed_for_dist(dist)
            br = math.radians(brg)
            vn = spd * math.cos(br)
            ve = spd * math.sin(br)
            aerr = tgt.alt - malt
            vd = -max(min(aerr * 1.5, 4.0), -4.0)
            return vn, ve, vd, self.phase

        else:
            self.phase = "RAM"
            self.prev_los = None
            self.prev_t = None
            brg = bearing_deg(mlat, mlon, tgt.lat, tgt.lon)
            br = math.radians(brg)
            vn = SPEED_RAM * math.cos(br)
            ve = SPEED_RAM * math.sin(br)
            aerr = tgt.alt - malt
            vd = -max(min(aerr * 2.0, 5.0), -5.0)
            return vn, ve, vd, self.phase

# ─── Drone Controller ────────────────────────────────────────────

class Drone:
    def __init__(self, conn_str):
        print(f"[CONN] Connecting to {conn_str} ...")
        self.mav = mavutil.mavlink_connection(conn_str)
        self.mav.wait_heartbeat()
        self.sid = self.mav.target_system
        self.cid = self.mav.target_component
        self.boot = time.time()
        print(f"[CONN] Heartbeat OK — system {self.sid}")

        self._request_streams()
        time.sleep(1)

        print(f"[CONN] Waiting for GPS ...")
        for i in range(60):
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg and msg.lat != 0:
                print(f"[CONN] GPS OK: {msg.lat/1e7:.6f}, {msg.lon/1e7:.6f}")
                return
            if i > 0 and i % 5 == 0:
                print(f"[CONN]   ... {i}s, re-requesting streams")
                self._request_streams()
        print("[CONN] WARNING: No GPS after 60s, continuing")

    def _request_streams(self):
        for s in range(13):
            self.mav.mav.request_data_stream_send(self.sid, self.cid, s, 10, 1)

    def _tms(self):
        return int((time.time() - self.boot) * 1000)

    def get_pos(self):
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if not msg:
            return None
        return {
            "lat": msg.lat / 1e7, "lon": msg.lon / 1e7,
            "alt": msg.relative_alt / 1000.0,
            "vx": msg.vx / 100.0, "vy": msg.vy / 100.0, "vz": msg.vz / 100.0,
        }

    def set_mode(self, name):
        modes = {"STABILIZE":0,"ACRO":1,"ALT_HOLD":2,"AUTO":3,
                 "GUIDED":4,"LOITER":5,"RTL":6,"LAND":9}
        mid = modes.get(name, 4)
        for _ in range(20):
            self.mav.set_mode_apm(mid)
            hb = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb and hb.custom_mode == mid:
                return True
        return False

    def arm_and_takeoff(self, alt_m):
        print(f"[MODE] GUIDED")
        self.set_mode("GUIDED")

        print(f"[ARM] Arming ...")
        self.mav.arducopter_arm()
        self.mav.motors_armed_wait()
        print(f"[ARM] Armed!")

        print(f"[TAKEOFF] Target {alt_m}m")
        self.mav.mav.command_long_send(
            self.sid, self.cid, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt_m)

        t0 = time.time()
        lp = 0
        while True:
            pos = self.get_pos()
            now = time.time()
            if pos:
                a = pos["alt"]
                if now - lp > 2:
                    print(f"[TAKEOFF] {a:.1f}m / {alt_m}m")
                    lp = now
                if a >= alt_m * 0.85:
                    print(f"[TAKEOFF] {a:.1f}m ✓ READY")
                    return
            else:
                if now - lp > 3:
                    print(f"[TAKEOFF] waiting for alt data ...")
                    self._request_streams()
                    lp = now
            if now - t0 > 60:
                print(f"[TAKEOFF] TIMEOUT — proceeding")
                return
            time.sleep(0.3)

    def send_vel(self, vn, ve, vd, yr=0.0):
        self.mav.mav.set_position_target_local_ned_send(
            self._tms(), self.sid, self.cid,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_TYPEMASK,
            0, 0, 0, vn, ve, vd, 0, 0, 0, 0, yr)

    def land(self):
        print("[RTL] Returning ...")
        self.set_mode("RTL")

# ─── Force Params ─────────────────────────────────────────────────

def force_params(mav):
    params = {
        "ATC_ANGLE_MAX": 70.0, "ATC_ANGLE_BOOST": 0.0,
        "WP_SPD": 40.0, "LOIT_SPEED_MS": 40.0, "LOIT_ACC_MAX_M": 15.0,
        "MOT_THST_HOVER": 0.125,
        "PSC_NE_VEL_P": 6.0, "PSC_NE_VEL_I": 2.0,
        "PSC_NE_POS_P": 2.0, "PSC_JERK_NE": 20.0,
    }
    print("\n  Setting params:")
    for k, v in params.items():
        mav.mav.param_set_send(mav.target_system, mav.target_component,
                                k.encode('utf-8'), v, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f"    {k} = {v}")
        time.sleep(0.1)
    time.sleep(0.5)
    print(f"  ✓ Velocity NED — no speed cap")
    print(f"  ✓ ATC_ANGLE_MAX=70°\n")

# ─── Logger ───────────────────────────────────────────────────────

class Logger:
    def __init__(self):
        self.f = open("intercept_v10_log.csv", "w")
        self.f.write("t,tgt,phase,dist,speed,vn,ve,vd,mlat,mlon,malt,tlat,tlon\n")

    def log(self, tid, ph, d, s, vn, ve, vd, ml, mo, ma, tl, to):
        self.f.write(f"{time.time():.3f},{tid},{ph},{d:.1f},{s:.1f},"
                     f"{vn:.2f},{ve:.2f},{vd:.2f},"
                     f"{ml:.7f},{mo:.7f},{ma:.1f},{tl:.7f},{to:.7f}\n")

    def close(self):
        self.f.close()

# ─── Mission ──────────────────────────────────────────────────────

def run(conn_str):
    drone = Drone(conn_str)
    force_params(drone.mav)

    signal.signal(signal.SIGINT, lambda s, f: (drone.land(), time.sleep(2), sys.exit(0)))

    drone.arm_and_takeoff(TAKEOFF_ALT_M)
    time.sleep(1)

    pos = drone.get_pos()
    if not pos:
        print("[ERROR] No position!")
        return

    targets = build_targets(pos["lat"], pos["lon"], INTERCEPT_ALT_M)
    for t in targets:
        t.reset_clock()
        d = haversine_m(pos["lat"], pos["lon"], t.lat, t.lon)
        print(f"  T#{t.id}: {d:.0f}m, spd={t.speed}m/s, hdg={t.course}°")

    guide = Guidance()
    log = Logger()
    active = list(targets)
    done = []
    peak = 0.0

    print()
    print("=" * 50)
    print("  INTERCEPT v10 — HUNTING")
    print("  PRONAV > PURSUIT > RAM")
    print("=" * 50)
    print()

    t0 = time.time()

    while active:
        pos = drone.get_pos()
        if not pos:
            time.sleep(0.01)
            continue

        ml, mo, ma = pos["lat"], pos["lon"], pos["alt"]
        vx, vy = pos["vx"], pos["vy"]
        gs = math.sqrt(vx**2 + vy**2)
        peak = max(peak, gs)

        for t in active:
            t.step()

        tgt = min(active, key=lambda t: haversine_m(ml, mo, t.lat, t.lon))
        dist = haversine_m(ml, mo, tgt.lat, tgt.lon)

        if dist <= CONTACT_M:
            el = time.time() - t0
            print(f"\n  *** INTERCEPT T#{tgt.id} d={dist:.1f}m "
                  f"GS={gs:.1f}m/s t={el:.1f}s ***\n")
            done.append(tgt.id)
            active.remove(tgt)
            guide.reset()
            continue

        vn, ve, vd, phase = guide.compute(ml, mo, ma, vx, vy, tgt)

        brg = bearing_deg(ml, mo, tgt.lat, tgt.lon)
        hdg = (math.degrees(math.atan2(vy, vx)) + 360) % 360
        yr = max(min(wrap_180(brg - hdg) * 0.05, 1.0), -1.0)

        drone.send_vel(vn, ve, vd, yr)

        cs = math.sqrt(vn**2 + ve**2)
        log.log(tgt.id, phase, dist, gs, vn, ve, vd, ml, mo, ma, tgt.lat, tgt.lon)
        print(f"  T#{tgt.id} [{phase:7s}] d={dist:6.1f}m GS={gs:5.1f} cmd={cs:5.1f} pk={peak:.1f}")

        time.sleep(GUIDANCE_DT_S)

    log.close()
    elapsed = time.time() - t0
    print()
    print("=" * 50)
    print(f"  DONE — {elapsed:.1f}s")
    print(f"  Intercepted: {done}")
    print(f"  Peak speed: {peak:.1f} m/s")
    print("=" * 50)

    drone.land()
    time.sleep(5)

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    run(p.parse_args().connect)
