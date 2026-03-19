# ============================================================
# DRONE INTERCEPTOR v8.1
# Uses simple_goto — proven to work in v3.0, v5.1, v6.0
# Clean architecture — no velocity conflicts
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
from datetime import datetime

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

# ============================================================
# CONSTANTS
# ============================================================
SAFE_ZONE     = 15
LOOP_RATE     = 0.5    # proven stable rate
INTERCEPT_ALT = 10
JAM_RANGE     = 15
JAM_SUCCESS   = 0.40
RAM_TRIGGER   = 3.0
TRACK_SPEED   = 10
RAM_SPEED     = 15
SPAWN_DIST    = 0.0003

# ============================================================
# GLOBALS
# ============================================================
sitl_process  = None
vehicle       = None
SOLDIER_POS   = None
log_entries   = []
log_start     = time.time()

report = {
    "enemies_detected" : 0,
    "jammed"           : 0,
    "rammed"           : 0,
    "escaped"          : 0,
    "soldier_safe"     : True,
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
        f"~/drone_interceptor/logs/v8_{ts}.json")
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
            vehicle.close()
    except:
        pass
    try:
        if sitl_process:
            os.killpg(os.getpgid(sitl_process.pid),
                      signal.SIGTERM)
    except:
        pass
    subprocess.run("pkill -f arducopter",
                   shell=True, capture_output=True)
    try:
        save_log()
    except:
        pass
    log_ok("Done!")
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# ============================================================
# SITL
# ============================================================
def kill_old_sitl():
    log_info("Clearing old processes...")
    subprocess.run("pkill -f arducopter",
                   shell=True, capture_output=True)
    subprocess.run("pkill -f sim_vehicle",
                   shell=True, capture_output=True)
    subprocess.run("fuser -k 5760/tcp",
                   shell=True, capture_output=True)
    time.sleep(3)
    log_ok("Cleared!")

def start_sitl():
    global sitl_process
    arducopter = os.path.expanduser(
        "~/ardupilot/build/sitl/bin/arducopter")
    defaults = os.path.expanduser(
        "~/ardupilot/Tools/autotest/"
        "default_params/copter.parm")
    if not os.path.exists(arducopter):
        log_err("arducopter not found!")
        sys.exit(1)
    log_info("Starting SITL...")
    sitl_process = subprocess.Popen(
        [arducopter,
         "--model",    "+",
         "--speedup",  "1",
         "--defaults", defaults,
         "--sim-address=127.0.0.1"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    log_ok("SITL started!")

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
    log_info("Connecting...")
    for attempt in range(3):
        try:
            vehicle = connect(
                'tcp:127.0.0.1:5760',
                wait_ready=True,
                timeout=60)
            log_ok("Connected!")
            log_info(f"Battery: {vehicle.battery.level}%")
            log_info(f"GPS: {vehicle.gps_0.fix_type}")
            return
        except Exception as e:
            log_warn(f"Attempt {attempt+1}: {e}")
            time.sleep(5)
    log_err("Failed!")
    cleanup()

# ============================================================
# MATH — HAVERSINE DISTANCE
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
    return (math.degrees(math.atan2(x, y)) + 360) % 360

# ============================================================
# MATH — KALMAN FILTER
# Initializes with first real GPS reading
# ============================================================
class KalmanFilter:
    def __init__(self):
        self.x = None
        self.v = 0.0
        self.P = 1.0
        self.Q = 0.001
        self.R = 0.05

    def update(self, z, dt=0.5):
        if self.x is None:
            self.x = z
            return z
        self.x += self.v * dt
        self.P += self.Q
        K       = self.P / (self.P + self.R)
        innov   = z - self.x
        self.x += K * innov
        self.v += K * (innov / max(dt, 0.01))
        self.P  = (1 - K) * self.P
        return self.x

# ============================================================
# MATH — PREDICTIVE INTERCEPT
# Fly to where enemy WILL BE
# ============================================================
def predict_intercept(my_pos, enemy, my_speed):
    dist    = get_distance(my_pos, enemy.position())
    e_speed = enemy.speed_ms()

    if my_speed <= 0 or dist <= 0:
        return enemy.lat, enemy.lon

    # Time to reach enemy
    t_reach = dist / max(my_speed, 1)

    # Predict ahead proportionally
    # Cap at 5 seconds to avoid overshoot
    t_pred   = min(t_reach * 0.5, 5.0)

    pred_lat = enemy.lat + enemy.dlat * t_pred * 50
    pred_lon = enemy.lon + enemy.dlon * t_pred * 50

    # Sanity check — prediction shouldn't be
    # more than 2x current distance away
    pred_loc = LocationGlobalRelative(
        pred_lat, pred_lon, INTERCEPT_ALT)
    pred_dist = get_distance(my_pos, pred_loc)

    if pred_dist > dist * 2:
        # Too far — just fly direct
        return enemy.lat, enemy.lon

    return pred_lat, pred_lon

# ============================================================
# MATH — RF JAMMING PHYSICS
# ============================================================
def jam_effectiveness(distance):
    if distance <= 0:
        return 1.0
    path_loss = (4 * math.pi * distance / 0.125)**2
    rx_power  = 10.0 / path_loss
    return min(1.0, rx_power / 1e-11)

def attempt_jam(distance):
    eff = jam_effectiveness(distance)
    log_action(f"Jamming... {eff*100:.0f}% effective")
    flight_log("JAM_ATTEMPT", {
        "dist": round(distance, 1),
        "eff":  round(eff, 2)
    })
    time.sleep(1.5)
    success = random.random() < min(
        JAM_SUCCESS, eff * 0.8)
    flight_log("JAM_RESULT", {"success": success})
    if success:
        log_ok("JAM SUCCESSFUL! ✅")
    else:
        log_warn("JAM FAILED! → RAM")
    return success

# ============================================================
# NAVIGATION — simple_goto ONLY
# Proven to work — no velocity conflicts
# ============================================================
def goto_target(lat, lon, alt, speed):
    target = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target, groundspeed=speed)
    report["max_speed"] = max(
        report["max_speed"], speed)

# ============================================================
# SAFETY
# ============================================================
def is_near_soldier(lat, lon):
    if SOLDIER_POS is None:
        return False
    dist = get_distance(
        SOLDIER_POS,
        LocationGlobalRelative(lat, lon, 0))
    if dist < SAFE_ZONE:
        log_safe(f"Safe zone! {dist:.1f}m!")
        return True
    return False

def check_battery():
    try:
        lvl = vehicle.battery.level
        if lvl is not None and lvl < 10:
            log_warn(f"Critical battery: {lvl}%")
            return True
    except:
        pass
    return False

# ============================================================
# ENEMY CLASS
# ============================================================
class Enemy:
    def __init__(self, id, lat, lon, soldier_pos):
        self.id          = id
        self.lat         = lat
        self.lon         = lon
        self.alt         = 8
        self.alive       = True
        self.soldier_pos = soldier_pos
        self.step        = 0
        self.evading     = False
        self.evade_cd    = 0
        self.prev_dlat   = 0
        self.prev_dlon   = 0
        self.skill       = random.uniform(0.4, 0.9)
        self.change_t    = random.randint(8, 15)

        # Direction toward soldier
        if soldier_pos:
            dlat = soldier_pos.lat - lat
            dlon = soldier_pos.lon - lon
            mag  = math.sqrt(dlat**2 + dlon**2)
            if mag > 0:
                spd       = random.uniform(
                    0.000003, 0.000005)
                self.dlat = (dlat / mag) * spd
                self.dlon = (dlon / mag) * spd
        else:
            self.dlat = random.uniform(
                -0.000004, 0.000004)
            self.dlon = random.uniform(
                -0.000004, 0.000004)

    def update(self, my_pos):
        if not self.alive:
            return

        self.step    += 1
        self.evade_cd = max(0, self.evade_cd - 1)
        self.prev_dlat = self.dlat
        self.prev_dlon = self.dlon

        # Evasion when interceptor close
        if my_pos and self.evade_cd == 0:
            my_loc = LocationGlobalRelative(
                my_pos.lat, my_pos.lon,
                INTERCEPT_ALT)
            dist   = get_distance(
                my_loc, self.position())
            if dist < 20:
                approach  = get_bearing(
                    my_loc, self.position())
                evade     = math.radians(
                    approach + 90 *
                    random.choice([-1, 1]))
                self.dlat = math.cos(evade) * 0.000006
                self.dlon = math.sin(evade) * 0.000006
                self.evade_cd = 8
                self.evading  = True
                log_threat(f"Enemy {self.id} evading!")
            else:
                self.evading = False

        # Random direction change
        if (self.step % self.change_t == 0 and
                not self.evading):
            self.change_t  = random.randint(8, 15)
            self.dlat += random.uniform(
                -0.000002, 0.000002)
            self.dlon += random.uniform(
                -0.000002, 0.000002)

        # Retarget soldier
        if (self.soldier_pos and
                self.step % 10 == 0 and
                not self.evading):
            dlat = self.soldier_pos.lat - self.lat
            dlon = self.soldier_pos.lon - self.lon
            mag  = math.sqrt(dlat**2 + dlon**2)
            if mag > 0:
                spd       = random.uniform(
                    0.000003, 0.000005)
                self.dlat = (dlat / mag) * spd
                self.dlon = (dlon / mag) * spd

        # Speed cap
        cap       = 0.000006
        self.dlat = max(-cap, min(cap, self.dlat))
        self.dlon = max(-cap, min(cap, self.dlon))
        self.lat += self.dlat
        self.lon += self.dlon

    def position(self):
        return LocationGlobalRelative(
            self.lat, self.lon, self.alt)

    def distance_to_soldier(self):
        if self.soldier_pos:
            return get_distance(
                self.soldier_pos, self.position())
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
        log_info(f"  GPS:{vehicle.gps_0.fix_type} | "
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
# ENGAGE ONE ENEMY
# Uses simple_goto — proven reliable
# ============================================================
def engage(enemy):
    k_lat      = KalmanFilter()
    k_lon      = KalmanFilter()
    jam_tried  = False
    dive_mode  = False
    start_time = time.time()
    timeout    = 120

    log_action(f"\n{'='*45}")
    log_action(f"Engaging Enemy {enemy.id}")
    log_action(f"Distance to soldier: "
              f"{enemy.distance_to_soldier():.0f}m")
    log_action(f"{'='*45}")
    flight_log("ENGAGE_START",
               {"enemy": enemy.id})

    while enemy.alive:

        # Timeout
        if time.time() - start_time > timeout:
            log_warn(f"Enemy {enemy.id} timeout!")
            return "escaped"

        # Battery
        if check_battery():
            return "escaped"

        # Get positions
        my_pos = vehicle.location\
                        .global_relative_frame

        # Update enemy
        enemy.update(my_pos)

        # Kalman smooth enemy position
        s_lat = k_lat.update(enemy.lat, LOOP_RATE)
        s_lon = k_lon.update(enemy.lon, LOOP_RATE)

        # Distance to smoothed position
        smooth_pos = LocationGlobalRelative(
            s_lat, s_lon, enemy.alt)
        distance   = get_distance(my_pos, smooth_pos)
        dist_sol   = enemy.distance_to_soldier()

        # Threat display
        sol_color = (f"{C.RED}{dist_sol:.0f}m{C.END}"
                    if dist_sol < 20
                    else f"{C.YELLOW}"
                         f"{dist_sol:.0f}m{C.END}")

        # Soldier danger
        if dist_sol < 15:
            log_threat(f"SOLDIER DANGER! "
                      f"Enemy {enemy.id} "
                      f"= {dist_sol:.0f}m!")
            report["soldier_safe"] = False
            flight_log("SOLDIER_DANGER", {
                "enemy": enemy.id,
                "dist":  round(dist_sol)
            })

        # =============================================
        # PHASE 1 — TRACK using simple_goto
        # =============================================
        if not dive_mode and distance > JAM_RANGE:

            # Predict where enemy will be
            pred_lat, pred_lon = predict_intercept(
                my_pos, enemy, TRACK_SPEED)

            # Fly to predicted position
            goto_target(pred_lat, pred_lon,
                       INTERCEPT_ALT, TRACK_SPEED)

            print(f"🎯 {C.MAGENTA}TRACK{C.END} | "
                  f"E{enemy.id} | "
                  f"Dist: {C.YELLOW}"
                  f"{distance:.1f}m{C.END} | "
                  f"Sol: {sol_color} | "
                  f"{'EVADE' if enemy.evading else 'APPROACH'}")

            flight_log("TRACKING", {
                "enemy":    enemy.id,
                "distance": round(distance, 1)
            })

        # =============================================
        # PHASE 2 — JAM
        # =============================================
        elif (not jam_tried and
              not dive_mode and
              distance <= JAM_RANGE):

            if is_near_soldier(enemy.lat, enemy.lon):
                log_safe("Near soldier — holding!")
            else:
                jam_tried = True
                success   = attempt_jam(distance)

                if success:
                    log_ok(f"ENEMY {enemy.id} "
                          f"JAMMED! ✅")
                    enemy.alive = False
                    report["jammed"] += 1
                    report["soldier_safe"] = True
                    flight_log("JAMMED", {
                        "enemy": enemy.id
                    })
                    return "jammed"
                else:
                    dive_mode = True

        # =============================================
        # PHASE 3 — RAM using simple_goto
        # =============================================
        elif dive_mode:

            if is_near_soldier(
                    enemy.lat, enemy.lon):
                log_safe("Near soldier! Re-jamming!")
                jam_tried = False
                dive_mode = False
            else:
                # Fly directly to enemy at RAM speed
                goto_target(enemy.lat, enemy.lon,
                           enemy.alt, RAM_SPEED)

                print(f"💥 {C.RED}RAM{C.END} | "
                      f"E{enemy.id} | "
                      f"Dist: {C.YELLOW}"
                      f"{distance:.1f}m{C.END} | "
                      f"{C.RED}{RAM_SPEED}m/s!{C.END}")

                # Trigger when close enough
                if distance < RAM_TRIGGER:
                    ke = int(0.5 * 0.5 * RAM_SPEED**2)
                    print(f"\n{C.RED}{C.BOLD}"
                          f"💥💥 ENEMY {enemy.id} "
                          f"DESTROYED!"
                          f"\nKE = {ke} Joules!"
                          f"{C.END}\n")
                    enemy.alive = False
                    report["rammed"] += 1
                    report["soldier_safe"] = True
                    flight_log("RAMMED", {
                        "enemy": enemy.id,
                        "ke":    ke
                    })
                    return "rammed"

        time.sleep(LOOP_RATE)

    return "done"

# ============================================================
# MISSION REPORT
# ============================================================
def print_report():
    dur   = time.time() - report["start_time"]
    total = report["enemies_detected"]
    neut  = report["jammed"] + report["rammed"]
    rate  = (neut / total * 100) if total > 0 else 0
    try:
        batt = vehicle.battery.level
    except:
        batt = "N/A"

    print(f"\n{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}{'📋 MISSION REPORT v8.1':^45}{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}")
    print(f"  ⏱️  Duration    : {dur:.0f}s")
    print(f"  🚨 Enemies     : {total}")
    print(f"{C.GREEN}  📡 Jammed      : "
          f"{report['jammed']}{C.END}")
    print(f"{C.YELLOW}  💥 Rammed      : "
          f"{report['rammed']}{C.END}")
    print(f"{C.RED}  ⚠️  Escaped     : "
          f"{report['escaped']}{C.END}")
    print(f"  🔋 Battery     : {batt}%")
    print(f"  🚀 Max speed   : "
          f"{report['max_speed']}m/s")
    clr = C.GREEN if rate == 100 else C.YELLOW
    print(f"{clr}  🎯 Success     : "
          f"{rate:.0f}%{C.END}")
    status = (f"{C.GREEN}SAFE ✅{C.END}"
              if report["soldier_safe"]
              else f"{C.RED}AT RISK ❌{C.END}")
    print(f"  🪖 Soldier     : {status}")
    print(f"{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}{'🪖 MISSION COMPLETE!':^45}{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}\n")

# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":

    print(f"\n{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}"
          f"{'🚁 INTERCEPTOR v8.1':^45}"
          f"{C.END}")
    print(f"{C.CYAN}"
          f"{'simple_goto | Kalman | Predictive':^45}"
          f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}\n")

    # Startup
    kill_old_sitl()
    start_sitl()
    wait_for_gps()
    connect_vehicle()

    print(f"\n{C.BOLD}--- MISSION START ---{C.END}\n")
    arm_and_takeoff(INTERCEPT_ALT)
    time.sleep(2)

    # Lock soldier
    home        = vehicle.location\
                         .global_relative_frame
    SOLDIER_POS = LocationGlobalRelative(
        home.lat, home.lon, 0)
    log_safe(f"Soldier: {home.lat:.6f}, "
            f"{home.lon:.6f}")
    log_safe(f"Safe zone: {SAFE_ZONE}m")
    flight_log("START", {
        "lat": home.lat,
        "lon": home.lon
    })

    # Spawn 2 enemies
    print(f"\n{C.RED}{C.BOLD}"
          f"⚠️  2 ENEMY DRONES INCOMING!{C.END}\n")

    enemy1 = Enemy(
        id=1,
        lat=home.lat + random.uniform(
            SPAWN_DIST, SPAWN_DIST * 1.5),
        lon=home.lon + random.uniform(
            -SPAWN_DIST, SPAWN_DIST),
        soldier_pos=SOLDIER_POS)

    enemy2 = Enemy(
        id=2,
        lat=home.lat + random.uniform(
            -SPAWN_DIST * 1.5, -SPAWN_DIST),
        lon=home.lon + random.uniform(
            -SPAWN_DIST, SPAWN_DIST),
        soldier_pos=SOLDIER_POS)

    enemies = [enemy1, enemy2]
    report["enemies_detected"] = 2

    for e in enemies:
        log_threat(f"Enemy {e.id} | "
                  f"{e.distance_to_soldier():.0f}m "
                  f"from soldier | "
                  f"speed={e.speed_ms():.1f}m/s")
        flight_log("ENEMY_SPAWNED", {
            "id":    e.id,
            "dist":  round(e.distance_to_soldier()),
            "speed": round(e.speed_ms(), 1)
        })

    print(f"\n{C.CYAN}"
          f"🔍 INTERCEPTOR v8.1 ONLINE{C.END}")
    print(f"{C.CYAN}"
          f"simple_goto | Kalman | "
          f"Predictive | RF Physics{C.END}\n")
    time.sleep(1)

    # Engage each enemy sequentially
    for enemy in enemies:
        if not enemy.alive:
            continue

        result = engage(enemy)

        if result == "escaped":
            report["escaped"] += 1
            log_warn(f"Enemy {enemy.id} escaped!")
        elif result != "done":
            log_ok(f"Enemy {enemy.id} "
                  f"{result.upper()}!")

        time.sleep(1)

    # Return home
    print(f"\n{C.GREEN}{C.BOLD}"
          f"🏠 Returning home...{C.END}")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(8)

    print_report()
    save_log()
    cleanup()
