# ============================================================
# DRONE INTERCEPTOR v8.3
# Fixed per expert feedback:
# ✅ True non-blocking FSM jam timer
# ✅ Local return logic (not global dict)
# ✅ Emergency brake on every exit
# ✅ Heartbeat failsafe
# ✅ Geofence radius check
# ✅ argparse for portability
# ✅ Military terms refactored
# ✅ Real velocity math
# ============================================================

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import random
import subprocess
import os
import signal
import sys
import json
import argparse
from datetime import datetime

# ============================================================
# ARGUMENT PARSER
# ============================================================
parser = argparse.ArgumentParser(
    description="Drone Interceptor v8.3")
parser.add_argument(
    '--connect',
    default='udp:127.0.0.1:14550',
    help='Connection string')
parser.add_argument(
    '--sitl-path',
    default=os.path.expanduser(
        '~/ardupilot/build/sitl/bin/arducopter'),
    help='Path to arducopter binary')
parser.add_argument(
    '--defaults',
    default=os.path.expanduser(
        '~/ardupilot/Tools/autotest/'
        'default_params/copter.parm'),
    help='Path to defaults file')
args = parser.parse_args()

# ============================================================
# CONFIG
# ============================================================
class Config:
    INTERCEPT_ALT   = 10
    SAFE_ZONE       = 15
    LOOP_RATE       = 0.1
    MAX_RADIUS      = 300
    HEARTBEAT_LIMIT = 2.0
    JAM_RANGE       = 50
    JAM_DURATION    = 1.5
    JAM_SUCCESS     = 0.40
    RAM_TRIGGER     = 3.0
    TRACK_SPEED     = 10
    RAM_SPEED       = 15
    ENEMY_SPEED_MIN = 0.000005
    ENEMY_SPEED_MAX = 0.000008
    SPAWN_DIST      = 0.0004
    MAVPROXY_OUT1   = "udp:127.0.0.1:14550"
    MAVPROXY_OUT2   = "udp:127.0.0.1:14551"

# ============================================================
# FSM STATES
# ============================================================
class State:
    IDLE      = "IDLE"
    SEARCH    = "SEARCH"
    TRACK     = "TRACK"
    JAM       = "JAM"
    INTERCEPT = "INTERCEPT"
    BRAKE     = "BRAKE"
    RTL       = "RTL"
    COMPLETE  = "COMPLETE"

# ============================================================
# COLORS
# ============================================================
class C:
    RED     = '\033[91m'
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    BLUE    = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN    = '\033[96m'
    BOLD    = '\033[1m'
    END     = '\033[0m'

def log_ok(msg):     print(f"{C.GREEN}✅ {msg}{C.END}")
def log_err(msg):    print(f"{C.RED}❌ {msg}{C.END}")
def log_warn(msg):   print(f"{C.YELLOW}⚠️  {msg}{C.END}")
def log_info(msg):   print(f"{C.CYAN}ℹ️  {msg}{C.END}")
def log_action(msg): print(f"{C.MAGENTA}🎯 {msg}{C.END}")
def log_threat(msg): print(f"{C.RED}{C.BOLD}🚨 {msg}{C.END}")
def log_safe(msg):   print(f"{C.GREEN}{C.BOLD}🛡️  {msg}{C.END}")
def log_state(msg):  print(f"{C.CYAN}{C.BOLD}🔄 {msg}{C.END}")

# ============================================================
# GLOBALS
# ============================================================
sitl_process     = None
mavproxy_process = None
vehicle          = None
SOLDIER_POS      = None
HOME_POS         = None
log_entries      = []
log_start        = time.time()
MISSION_FAILED   = False

report = {
    "targets_detected" : 0,
    "jammed"           : 0,
    "intercepted"      : 0,
    "escaped"          : 0,
    "vip_safe"         : True,
    "start_time"       : time.time(),
    "max_speed"        : 0
}

# ============================================================
# LOGGER
# ============================================================
def flight_log(event, data={}):
    log_entries.append({
        "time":  round(time.time() - log_start, 2),
        "event": event,
        "data":  data
    })

def save_log():
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.expanduser(
        f"~/drone_interceptor/logs/v83_{ts}.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(log_entries, f, indent=2)
    log_info(f"Log saved: {path}")

# ============================================================
# CLEANUP
# ============================================================
def cleanup(signum=None, frame=None):
    print(f"\n{C.YELLOW}Cleaning up...{C.END}")
    try:
        if vehicle:
            vehicle.mode = VehicleMode("BRAKE")
            time.sleep(1)
            vehicle.close()
    except:
        pass
    try:
        if sitl_process:
            os.killpg(
                os.getpgid(sitl_process.pid),
                signal.SIGTERM)
    except:
        pass
    try:
        if mavproxy_process:
            os.killpg(
                os.getpgid(mavproxy_process.pid),
                signal.SIGTERM)
    except:
        pass
    subprocess.run("pkill -f arducopter",
                   shell=True,
                   capture_output=True)
    subprocess.run("pkill -f mavproxy",
                   shell=True,
                   capture_output=True)
    try:
        save_log()
    except:
        pass
    log_ok("Done!")
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# ============================================================
# SITL + MAVPROXY
# ============================================================
def kill_old_processes():
    log_info("Clearing old processes...")
    subprocess.run("pkill -f arducopter",
                   shell=True,
                   capture_output=True)
    subprocess.run("pkill -f mavproxy",
                   shell=True,
                   capture_output=True)
    subprocess.run("fuser -k 5760/tcp",
                   shell=True,
                   capture_output=True)
    time.sleep(3)
    log_ok("Cleared!")

def start_sitl():
    global sitl_process
    if not os.path.exists(args.sitl_path):
        log_err(f"SITL not found: "
               f"{args.sitl_path}")
        sys.exit(1)
    log_info("Starting SITL...")
    sitl_process = subprocess.Popen(
        [args.sitl_path,
         "--model",    "+",
         "--speedup",  "1",
         "--defaults", args.defaults,
         "--sim-address=127.0.0.1"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    log_ok("SITL started!")
    time.sleep(3)

def start_mavproxy():
    global mavproxy_process
    log_info("Starting MAVProxy...")
    mavproxy_process = subprocess.Popen(
        ["mavproxy.py",
         "--master=tcp:127.0.0.1:5760",
         f"--out={Config.MAVPROXY_OUT1}",
         f"--out={Config.MAVPROXY_OUT2}",
         "--streamrate=10"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    time.sleep(3)
    log_ok("MAVProxy started!")
    log_ok("Mission Planner → UDP 14551 ✅")

def wait_for_gps():
    log_info("Waiting 90 seconds for GPS...")
    for i in range(90, 0, -1):
        print(f"\r{C.CYAN}  ⏳ {i}s...{C.END}",
              end="", flush=True)
        time.sleep(1)
    print()
    log_ok("GPS ready!")

def connect_vehicle():
    global vehicle
    log_info(f"Connecting to {args.connect}...")
    for attempt in range(5):
        try:
            vehicle = connect(
                args.connect,
                wait_ready=True,
                timeout=60)
            log_ok("Connected!")
            log_info(
                f"Battery: "
                f"{vehicle.battery.level}%")
            return
        except Exception as e:
            log_warn(f"Attempt {attempt+1}: {e}")
            time.sleep(5)
    log_err("Connection failed!")
    cleanup()

# ============================================================
# MATH — HAVERSINE
# ============================================================
def get_distance(loc1, loc2):
    R    = 6371000
    lat1 = math.radians(loc1.lat)
    lat2 = math.radians(loc2.lat)
    dlat = math.radians(loc2.lat - loc1.lat)
    dlon = math.radians(loc2.lon - loc1.lon)
    a    = (math.sin(dlat/2)**2 +
            math.cos(lat1) *
            math.cos(lat2) *
            math.sin(dlon/2)**2)
    return 6371000 * 2 * math.atan2(
        math.sqrt(a), math.sqrt(1-a))

def get_bearing(loc1, loc2):
    lat1 = math.radians(loc1.lat)
    lat2 = math.radians(loc2.lat)
    dlon = math.radians(loc2.lon - loc1.lon)
    x    = math.sin(dlon) * math.cos(lat2)
    y    = (math.cos(lat1) * math.sin(lat2) -
            math.sin(lat1) * math.cos(lat2) *
            math.cos(dlon))
    return (math.degrees(
        math.atan2(x, y)) + 360) % 360

# ============================================================
# MATH — KALMAN FILTER
# ============================================================
class KalmanFilter:
    def __init__(self):
        self.x = None
        self.v = 0.0
        self.P = 1.0
        self.Q = 0.001
        self.R = 0.05

    def update(self, z, dt=0.1):
        if self.x is None:
            self.x = z
            return z
        self.x += self.v * dt
        self.P += self.Q
        K       = self.P / (self.P + self.R)
        innov   = z - self.x
        self.x += K * innov
        self.v += K * (innov / max(dt, 0.001))
        self.P  = (1 - K) * self.P
        return self.x

# ============================================================
# MATH — PREDICTIVE INTERCEPT
# Real m/s velocity — no magic numbers
# ============================================================
def predict_intercept(my_pos, target,
                      my_speed, dt):
    dist = get_distance(my_pos,
                       target.position())

    if my_speed <= 0 or dist <= 0:
        return target.lat, target.lon

    # Real velocity in m/s using dt
    vel_lat_ms = (target.dlat / dt) * 111320
    vel_lon_ms = (target.dlon / dt) * (
        111320 * math.cos(
            math.radians(target.lat)))

    # Time to intercept
    t_intercept = min(dist /
                     max(my_speed, 1), 8.0)

    # Predict using real physics
    pred_lat = (target.lat +
               (vel_lat_ms * t_intercept)
               / 111320)
    pred_lon = (target.lon +
               (vel_lon_ms * t_intercept) /
               (111320 * math.cos(
                   math.radians(target.lat))))

    # Sanity check
    pred_loc  = LocationGlobalRelative(
        pred_lat, pred_lon,
        Config.INTERCEPT_ALT)
    pred_dist = get_distance(my_pos, pred_loc)

    if pred_dist > dist * 2.0:
        return target.lat, target.lon

    return pred_lat, pred_lon

# ============================================================
# MATH — RF JAMMING PHYSICS
# ============================================================
def jam_effectiveness(distance):
    if distance <= 0:
        return 1.0
    path_loss = (4 * math.pi *
                 distance / 0.125)**2
    return min(1.0, 10.0 / path_loss / 1e-11)

# ============================================================
# FAILSAFES
# ============================================================
def emergency_brake():
    try:
        vehicle.mode = VehicleMode("BRAKE")
        time.sleep(0.5)
        log_safe("BRAKE applied!")
        flight_log("BRAKE_APPLIED")
    except Exception as e:
        log_err(f"BRAKE failed: {e}")

def check_heartbeat():
    try:
        hb = vehicle.last_heartbeat
        if hb > Config.HEARTBEAT_LIMIT:
            log_err(f"Heartbeat lost! "
                   f"{hb:.1f}s → RTL!")
            vehicle.mode = VehicleMode("RTL")
            flight_log("HEARTBEAT_LOST",
                       {"last_hb": hb})
            return False
    except Exception as e:
        log_err(f"Heartbeat error: {e}")
        return False
    return True

def check_geofence(my_pos):
    if HOME_POS is None:
        return True
    dist = get_distance(HOME_POS, my_pos)
    if dist > Config.MAX_RADIUS:
        log_err(f"GEOFENCE BREACH! "
               f"{dist:.0f}m → RTL!")
        vehicle.mode = VehicleMode("RTL")
        flight_log("GEOFENCE_BREACH",
                   {"dist": round(dist)})
        return False
    return True

def check_battery():
    try:
        lvl = vehicle.battery.level
        if lvl is not None and lvl < 10:
            log_warn(f"Critical battery! RTL!")
            vehicle.mode = VehicleMode("RTL")
            return False
    except:
        pass
    return True

def run_all_failsafes(my_pos):
    if not check_heartbeat():
        return False
    if not check_geofence(my_pos):
        return False
    if not check_battery():
        return False
    return True

# ============================================================
# NAVIGATION
# ============================================================
def goto_target(lat, lon, alt, speed):
    target = LocationGlobalRelative(
        lat, lon, alt)
    vehicle.simple_goto(target,
                       groundspeed=speed)
    report["max_speed"] = max(
        report["max_speed"], speed)

# ============================================================
# VIP SAFE ZONE CHECK
# ============================================================
def near_vip(lat, lon):
    if SOLDIER_POS is None:
        return False
    dist = get_distance(
        SOLDIER_POS,
        LocationGlobalRelative(lat, lon, 0))
    if dist < Config.SAFE_ZONE:
        log_safe(f"VIP safe zone! {dist:.1f}m!")
        return True
    return False

# ============================================================
# TARGET CLASS
# ============================================================
class Target:
    def __init__(self, id, lat, lon, vip_pos):
        self.id          = id
        self.lat         = lat
        self.lon         = lon
        self.alt         = 8
        self.active      = True
        self.vip_pos     = vip_pos
        self.step        = 0
        self.evading     = False
        self.evade_cd    = 0
        self.skill       = random.uniform(0.4, 0.9)
        self.change_t    = random.randint(8, 15)
        self.last_update = time.time()
        self.dt          = Config.LOOP_RATE

        if vip_pos:
            dlat = vip_pos.lat - lat
            dlon = vip_pos.lon - lon
            mag  = math.sqrt(dlat**2 + dlon**2)
            if mag > 0:
                spd = random.uniform(
                    Config.ENEMY_SPEED_MIN,
                    Config.ENEMY_SPEED_MAX)
                self.dlat = (dlat / mag) * spd
                self.dlon = (dlon / mag) * spd
        else:
            spd       = Config.ENEMY_SPEED_MAX
            self.dlat = random.uniform(-spd, spd)
            self.dlon = random.uniform(-spd, spd)

    def update(self, my_pos):
        if not self.active:
            return

        now           = time.time()
        self.dt       = max(
            now - self.last_update, 0.001)
        self.last_update = now
        self.step    += 1
        self.evade_cd = max(0,
                           self.evade_cd - 1)

        if my_pos and self.evade_cd == 0:
            my_loc = LocationGlobalRelative(
                my_pos.lat, my_pos.lon,
                Config.INTERCEPT_ALT)
            dist   = get_distance(
                my_loc, self.position())
            if dist < 25:
                approach  = get_bearing(
                    my_loc, self.position())
                evade     = math.radians(
                    approach + 90 *
                    random.choice([-1, 1]))
                spd       = Config.ENEMY_SPEED_MAX
                self.dlat = math.cos(evade) * spd
                self.dlon = math.sin(evade) * spd
                self.evade_cd = 8
                self.evading  = True
                log_threat(
                    f"Target {self.id} evading!")
            else:
                self.evading = False

        if (self.step % self.change_t == 0
                and not self.evading):
            self.change_t = random.randint(8, 15)
            self.dlat    += random.uniform(
                -0.000003, 0.000003)
            self.dlon    += random.uniform(
                -0.000003, 0.000003)

        if (self.vip_pos and
                self.step % 10 == 0 and
                not self.evading):
            dlat = self.vip_pos.lat - self.lat
            dlon = self.vip_pos.lon - self.lon
            mag  = math.sqrt(dlat**2 + dlon**2)
            if mag > 0:
                spd = random.uniform(
                    Config.ENEMY_SPEED_MIN,
                    Config.ENEMY_SPEED_MAX)
                self.dlat = (dlat / mag) * spd
                self.dlon = (dlon / mag) * spd

        cap       = Config.ENEMY_SPEED_MAX * 1.2
        self.dlat = max(-cap, min(cap, self.dlat))
        self.dlon = max(-cap, min(cap, self.dlon))
        self.lat += self.dlat
        self.lon += self.dlon

        if self.distance_to_vip() < 2:
            global MISSION_FAILED
            MISSION_FAILED = True
            log_threat(
                f"🚨 TARGET {self.id} "
                f"REACHED VIP! "
                f"MISSION FAILED!")
            report["vip_safe"] = False

    def position(self):
        return LocationGlobalRelative(
            self.lat, self.lon, self.alt)

    def distance_to_vip(self):
        if self.vip_pos:
            return get_distance(
                self.vip_pos, self.position())
        return 999

    def speed_ms(self):
        return (math.sqrt(
            self.dlat**2 + self.dlon**2) *
            1.113195e5)

# ============================================================
# ARM AND TAKEOFF
# ============================================================
def arm_and_takeoff(alt):
    log_info("Waiting for armable...")
    start = time.time()
    while not vehicle.is_armable:
        if time.time() - start > 60:
            log_err("Timeout!")
            cleanup()
        log_info(
            f"  GPS:"
            f"{vehicle.gps_0.fix_type} | "
            f"Sats:"
            f"{vehicle.gps_0.satellites_visible}")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.5)
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    log_ok("Armed!")

    vehicle.simple_takeoff(alt)
    while True:
        cur = vehicle.location\
                     .global_relative_frame.alt
        log_info(f"  Alt: {cur:.1f}m / {alt}m")
        if cur >= alt * 0.95:
            log_ok(f"Reached {alt}m!")
            break
        time.sleep(1)

# ============================================================
# ENGAGE — TRUE FSM
# Non-blocking jam timer in main loop
# Local return logic only
# ============================================================
def engage(target):
    global MISSION_FAILED

    k_lat          = KalmanFilter()
    k_lon          = KalmanFilter()
    state          = State.SEARCH
    jam_start_time = 0
    timeout        = 120
    start          = time.time()

    log_state(f"\n{'='*45}")
    log_state(f"FSM START | Target {target.id} | "
             f"Speed: {target.speed_ms():.1f}m/s")
    log_state(f"{'='*45}")
    flight_log("FSM_START",
               {"target": target.id})

    while (target.active and
           not MISSION_FAILED and
           state not in [State.RTL]):

        # ── TIMEOUT ───────────────────────
        if time.time() - start > timeout:
            log_warn(f"Timeout! "
                    f"Target {target.id}!")
            emergency_brake()
            return "escaped"

        # ── POSITIONS ─────────────────────
        my_pos = vehicle.location\
                        .global_relative_frame

        # ── ALL FAILSAFES ─────────────────
        if not run_all_failsafes(my_pos):
            emergency_brake()
            return "escaped"

        # ── UPDATE TARGET ─────────────────
        target.update(my_pos)
        if MISSION_FAILED:
            emergency_brake()
            return "escaped"

        # ── KALMAN SMOOTH ─────────────────
        s_lat  = k_lat.update(
            target.lat, Config.LOOP_RATE)
        s_lon  = k_lon.update(
            target.lon, Config.LOOP_RATE)
        smooth = LocationGlobalRelative(
            s_lat, s_lon, target.alt)

        # ── DISTANCES ─────────────────────
        distance = get_distance(my_pos, smooth)
        dist_vip = target.distance_to_vip()

        # ── VIP STATUS ────────────────────
        vip_str = (
            f"{C.RED}{dist_vip:.0f}m{C.END}"
            if dist_vip < 30
            else
            f"{C.YELLOW}{dist_vip:.0f}m{C.END}")

        if dist_vip < 15:
            log_threat(
                f"VIP DANGER! "
                f"Target {target.id} = "
                f"{dist_vip:.0f}m!")
            report["vip_safe"] = False
            flight_log("VIP_DANGER", {
                "target": target.id,
                "dist":   round(dist_vip)
            })

        # ════════════════════════════════
        # FSM STATES
        # ════════════════════════════════

        # ── SEARCH ────────────────────────
        if state == State.SEARCH:
            log_state(
                f"SEARCH → TRACK | "
                f"Target {target.id} | "
                f"{distance:.0f}m")
            state = State.TRACK
            flight_log("STATE_CHANGE",
                       {"state": State.TRACK})

        # ── TRACK ─────────────────────────
        elif state == State.TRACK:

            if distance <= Config.JAM_RANGE:
                state          = State.JAM
                jam_start_time = time.time()
                log_state(
                    f"TRACK → JAM | "
                    f"{distance:.0f}m")
                flight_log("STATE_CHANGE",
                           {"state": State.JAM})
                continue

            dt = target.dt
            pred_lat, pred_lon = \
                predict_intercept(
                    my_pos, target,
                    Config.TRACK_SPEED, dt)

            goto_target(
                pred_lat, pred_lon,
                Config.INTERCEPT_ALT,
                Config.TRACK_SPEED)

            print(
                f"🎯 {C.MAGENTA}TRACK{C.END} | "
                f"T{target.id} | "
                f"Dist:{C.YELLOW}"
                f"{distance:.1f}m{C.END} | "
                f"VIP:{vip_str} | "
                f"{'EVADE' if target.evading else 'APPROACH'}")

            flight_log("TRACKING", {
                "target":   target.id,
                "distance": round(distance, 1)
            })

        # ── JAM (True Non-Blocking) ────────
        elif state == State.JAM:

            if near_vip(target.lat, target.lon):
                log_safe("Near VIP — holding!")
                state = State.TRACK
                continue

            # Slowly track during jam attempt
            # Main loop keeps running!
            goto_target(
                target.lat, target.lon,
                Config.INTERCEPT_ALT, 2)

            elapsed   = (time.time() -
                        jam_start_time)
            remaining = max(
                0, Config.JAM_DURATION - elapsed)

            print(
                f"📡 {C.BLUE}JAM{C.END} | "
                f"T{target.id} | "
                f"Dist:{C.YELLOW}"
                f"{distance:.1f}m{C.END} | "
                f"⏱️ {remaining:.1f}s")

            # Evaluate AFTER duration passes
            if elapsed >= Config.JAM_DURATION:
                eff     = jam_effectiveness(
                    distance)
                success = (random.random() <
                          min(Config.JAM_SUCCESS,
                              eff * 0.8))

                flight_log("JAM_RESULT", {
                    "success": success,
                    "eff":     round(eff, 2)
                })

                if success:
                    log_ok(
                        f"Target {target.id} "
                        f"NEUTRALISED — JAM ✅")
                    target.active  = False
                    report["jammed"] += 1
                    report["vip_safe"] = True
                    emergency_brake()
                    flight_log("JAM_SUCCESS", {
                        "target": target.id
                    })
                    return "jammed"  # Local!

                else:
                    log_warn(
                        "JAM FAILED! "
                        "→ PHYSICAL INTERCEPT")
                    state = State.INTERCEPT
                    flight_log("STATE_CHANGE", {
                        "state": State.INTERCEPT
                    })

        # ── INTERCEPT (Physical) ───────────
        elif state == State.INTERCEPT:

            if near_vip(target.lat, target.lon):
                log_safe(
                    "Near VIP! Re-attempting jam!")
                state          = State.JAM
                jam_start_time = time.time()
                continue

            goto_target(
                target.lat, target.lon,
                target.alt,
                Config.RAM_SPEED)

            print(
                f"💥 {C.RED}INTERCEPT{C.END} | "
                f"T{target.id} | "
                f"Dist:{C.YELLOW}"
                f"{distance:.1f}m{C.END} | "
                f"{C.RED}"
                f"{Config.RAM_SPEED}m/s!{C.END}")

            if distance < Config.RAM_TRIGGER:
                ke = int(0.5 * 0.5 *
                        Config.RAM_SPEED**2)
                print(
                    f"\n{C.RED}{C.BOLD}"
                    f"💥 TARGET {target.id} "
                    f"KINETIC MITIGATION!"
                    f"\nKE = {ke} Joules"
                    f"{C.END}\n")
                target.active  = False
                report["intercepted"] += 1
                report["vip_safe"]    = True
                emergency_brake()
                flight_log(
                    "KINETIC_MITIGATION", {
                        "target": target.id,
                        "ke":     ke
                    })
                return "intercepted"  # Local!

        time.sleep(Config.LOOP_RATE)

    return "escaped"

# ============================================================
# MISSION REPORT
# ============================================================
def print_report():
    dur   = time.time() - report["start_time"]
    total = report["targets_detected"]
    neut  = (report["jammed"] +
             report["intercepted"])
    rate  = (neut / total * 100) if total > 0 else 0
    try:
        batt = vehicle.battery.level
    except:
        batt = "N/A"

    print(f"\n{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}"
          f"{'📋 MISSION REPORT v8.3':^45}"
          f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}")
    print(f"  ⏱️  Duration       : {dur:.0f}s")
    print(f"  🎯 Targets         : {total}")
    print(f"{C.GREEN}  📡 Jammed          : "
          f"{report['jammed']}{C.END}")
    print(f"{C.YELLOW}  💥 Intercepted     : "
          f"{report['intercepted']}{C.END}")
    print(f"{C.RED}  ⚠️  Escaped         : "
          f"{report['escaped']}{C.END}")
    print(f"  🔋 Battery         : {batt}%")
    print(f"  🚀 Max speed       : "
          f"{report['max_speed']}m/s")
    clr = C.GREEN if rate == 100 else C.YELLOW
    print(f"{clr}  🎯 Success         : "
          f"{rate:.0f}%{C.END}")
    status = (
        f"{C.GREEN}SECURE ✅{C.END}"
        if report["vip_safe"]
        else f"{C.RED}COMPROMISED ❌{C.END}")
    print(f"  🪖 VIP Status      : {status}")
    if MISSION_FAILED:
        print(f"\n{C.RED}{C.BOLD}"
              f"  ❌ MISSION FAILED — "
              f"VIP COMPROMISED"
              f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}"
          f"{'🪖 MISSION COMPLETE v8.3!':^45}"
          f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}\n")

# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":

    print(f"\n{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}"
          f"{'🚁 INTERCEPTOR v8.3':^45}"
          f"{C.END}")
    print(f"{C.CYAN}"
          f"{'FSM | Failsafes | Non-Blocking':^45}"
          f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}\n")

    kill_old_processes()
    start_sitl()
    start_mavproxy()

    print(f"\n{C.GREEN}{C.BOLD}"
          f"📡 Mission Planner:"
          f"{C.END}")
    print(f"{C.GREEN}"
          f"  UDP → Port 14551 → Connect"
          f"{C.END}\n")

    wait_for_gps()
    connect_vehicle()

    print(f"\n{C.BOLD}"
          f"--- MISSION START ---{C.END}\n")
    arm_and_takeoff(Config.INTERCEPT_ALT)
    time.sleep(2)

    home_frame  = vehicle.location\
                         .global_relative_frame
    HOME_POS    = LocationGlobalRelative(
        home_frame.lat, home_frame.lon, 0)
    SOLDIER_POS = LocationGlobalRelative(
        home_frame.lat, home_frame.lon, 0)

    log_safe(f"VIP/Home: "
            f"{home_frame.lat:.6f}, "
            f"{home_frame.lon:.6f}")
    log_safe(f"VIP safe zone: {Config.SAFE_ZONE}m")
    log_safe(f"Geofence: {Config.MAX_RADIUS}m")
    log_safe(f"Heartbeat limit: "
            f"{Config.HEARTBEAT_LIMIT}s")

    flight_log("MISSION_START", {
        "lat":      home_frame.lat,
        "lon":      home_frame.lon,
        "geofence": Config.MAX_RADIUS
    })

    print(f"\n{C.RED}{C.BOLD}"
          f"⚠️  2 TARGETS INBOUND!{C.END}\n")

    target1 = Target(
        id=1,
        lat=home_frame.lat + random.uniform(
            Config.SPAWN_DIST,
            Config.SPAWN_DIST * 1.5),
        lon=home_frame.lon + random.uniform(
            -Config.SPAWN_DIST,
            Config.SPAWN_DIST),
        vip_pos=SOLDIER_POS)

    target2 = Target(
        id=2,
        lat=home_frame.lat + random.uniform(
            -Config.SPAWN_DIST * 1.5,
            -Config.SPAWN_DIST),
        lon=home_frame.lon + random.uniform(
            -Config.SPAWN_DIST,
            Config.SPAWN_DIST),
        vip_pos=SOLDIER_POS)

    targets = [target1, target2]
    report["targets_detected"] = 2

    for t in targets:
        log_threat(
            f"Target {t.id} | "
            f"{t.distance_to_vip():.0f}m | "
            f"skill={t.skill:.1f} | "
            f"speed={t.speed_ms():.1f}m/s")
        flight_log("TARGET_DETECTED", {
            "id":    t.id,
            "dist":  round(t.distance_to_vip()),
            "speed": round(t.speed_ms(), 1)
        })

    print(f"\n{C.CYAN}"
          f"🔍 INTERCEPTOR v8.3 ONLINE{C.END}")
    print(f"{C.CYAN}"
          f"FSM | Heartbeat | Geofence | "
          f"Non-blocking | Real Math{C.END}\n")
    time.sleep(1)

    for target in targets:
        if not target.active or MISSION_FAILED:
            continue

        result = engage(target)

        if MISSION_FAILED:
            log_threat("MISSION FAILED! "
                      "VIP COMPROMISED!")
            break
        elif result == "escaped":
            report["escaped"] += 1
            log_warn(
                f"Target {target.id} escaped!")
        else:
            log_ok(
                f"Target {target.id} "
                f"{result.upper()}!")

        time.sleep(0.5)

    if not MISSION_FAILED:
        print(f"\n{C.GREEN}{C.BOLD}"
              f"🏠 All targets handled! "
              f"Returning home...{C.END}")

    vehicle.mode = VehicleMode("RTL")
    time.sleep(8)

    print_report()
    save_log()
    cleanup()
