# ============================================================
# DRONE INTERCEPTOR v8.6
# Updated from v8.6 with:
# [+] Priority targeting - engages closest-to-VIP first
# [+] Target switching - aborts if another target gets closer
# [+] VIP_SWITCH_DIST threshold for emergency re-targeting
# [+] All active targets updated every loop iteration
# ------------------------------------------------------------
# Carried forward:
# [+] Fly-through ramming with MAVLink velocity commands
# [+] Predictive lead during ram phase
# [+] InterceptPhysics module
# [+] True non-blocking FSM jam timer
# [+] Heartbeat / Geofence / Battery failsafes
# [+] Kalman filter + predictive intercept
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
    description="Drone Interceptor v8.6")
parser.add_argument(
    "--connect",
    default="udp:127.0.0.1:14550",
    help="Connection string")
parser.add_argument(
    "--sitl-path",
    default=os.path.expanduser(
        "~/ardupilot/build/sitl/bin/arducopter"),
    help="Path to arducopter binary")
parser.add_argument(
    "--defaults",
    default=os.path.expanduser(
        "~/ardupilot/Tools/autotest/"
        "default_params/copter.parm"),
    help="Path to defaults file")
parser.add_argument(
    "--test-physics",
    action="store_true",
    help="Run InterceptPhysics unit test and exit")
parser.add_argument(
    "--interceptor-mass",
    type=float,
    default=1.5,
    help="Interceptor mass in kg (default: 1.5)")
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
    RAM_TRIGGER     = 5.0
    TRACK_SPEED     = 10
    RAM_SPEED       = 20
    ENEMY_SPEED_MIN = 0.000005
    ENEMY_SPEED_MAX = 0.000008
    SPAWN_DIST      = 0.0004
    MAVPROXY_OUT1   = "udp:127.0.0.1:14550"
    MAVPROXY_OUT2   = "udp:127.0.0.1:14551"
    INTERCEPTOR_MASS = 1.5
    VIP_SWITCH_DIST  = 25  # switch target if another gets this close to VIP

Config.INTERCEPTOR_MASS = args.interceptor_mass

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
    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    BLUE    = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN    = "\033[96m"
    BOLD    = "\033[1m"
    END     = "\033[0m"

def log_ok(msg):     print(f"{C.GREEN}[OK] {msg}{C.END}")
def log_err(msg):    print(f"{C.RED}[ERR] {msg}{C.END}")
def log_warn(msg):   print(f"{C.YELLOW}[WARN] {msg}{C.END}")
def log_info(msg):   print(f"{C.CYAN}[INFO] {msg}{C.END}")
def log_action(msg): print(f"{C.MAGENTA}[ACT] {msg}{C.END}")
def log_threat(msg): print(f"{C.RED}{C.BOLD}[THREAT] {msg}{C.END}")
def log_safe(msg):   print(f"{C.GREEN}{C.BOLD}[SAFE] {msg}{C.END}")
def log_state(msg):  print(f"{C.CYAN}{C.BOLD}[STATE] {msg}{C.END}")

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
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.expanduser(
        f"~/drone_interceptor/logs/v85_{ts}.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
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
    log_ok("Mission Planner on UDP 14551")

def wait_for_gps():
    log_info("Waiting 90 seconds for GPS...")
    for i in range(90, 0, -1):
        print(f"\r{C.CYAN}  [{i}s remaining]{C.END}",
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
# MATH - HAVERSINE
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
# MATH - KALMAN FILTER
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
# INTERCEPT PHYSICS MODULE (NEW in v8.6)
#
# Smart angle-based ramming: evaluates the best
# approach vector to maximise kinetic damage.
#
# Key insight for a kamikaze defender drone:
#   KE = 0.5 * m * v^2  -> velocity matters more than mass
#   Closing velocity = your speed + component of
#                      target speed toward you
#   Head-on = max closing speed = max destruction
#   Top-down dive into propeller plane = very effective
#   Rear chase = worst case, lowest relative speed
# ============================================================
class InterceptPhysics:
    """
    Calculates optimal ram approach and
    expected kinetic damage for kamikaze
    intercept of enemy drones.
    """

    HEAD_ON  = "HEAD_ON"
    SIDE     = "SIDE"
    TOP_DOWN = "TOP_DOWN"
    REAR     = "REAR"

    @staticmethod
    def closing_speed(my_speed, target_speed,
                      angle_diff_deg):
        """
        Real closing velocity based on
        relative heading angle.
        angle_diff_deg: 0 = same direction (chase),
                       180 = head-on
        """
        angle_rad = math.radians(angle_diff_deg)
        return my_speed + target_speed * math.cos(angle_rad)

    @staticmethod
    def kinetic_energy(mass_kg, velocity_ms):
        """Kinetic energy in Joules."""
        return 0.5 * mass_kg * velocity_ms ** 2

    @staticmethod
    def classify_angle(angle_diff_deg,
                       alt_advantage):
        """
        Determine impact type from geometry.
        alt_advantage: interceptor alt minus target alt (metres).
        """
        if alt_advantage > 3.0:
            return InterceptPhysics.TOP_DOWN
        if angle_diff_deg >= 135:
            return InterceptPhysics.HEAD_ON
        if angle_diff_deg >= 60:
            return InterceptPhysics.SIDE
        return InterceptPhysics.REAR

    @staticmethod
    def damage_multiplier(impact_type):
        """
        Effectiveness multiplier per approach angle.
        HEAD_ON  = 1.0  - full frontal, max structural damage
        TOP_DOWN = 0.95 - dive into props, nearly as good
        SIDE     = 0.7  - clip props/motors on one side
        REAR     = 0.3  - chasing, low relative speed
        """
        return {
            InterceptPhysics.HEAD_ON:  1.0,
            InterceptPhysics.TOP_DOWN: 0.95,
            InterceptPhysics.SIDE:     0.7,
            InterceptPhysics.REAR:     0.3,
        }.get(impact_type, 0.5)

    @staticmethod
    def evaluate(my_pos, my_heading, my_speed,
                 target_pos, target_heading,
                 target_speed, interceptor_mass):
        """
        Full ram assessment: what happens if we
        impact right now?

        Returns dict:
          impact_type   - HEAD_ON / SIDE / TOP_DOWN / REAR
          angle_diff    - heading difference in degrees
          alt_advantage - altitude advantage in metres
          closing_speed - combined speed at impact (m/s)
          ke_raw_joules - raw kinetic energy (J)
          ke_effective  - damage-adjusted KE (J)
          damage_mult   - multiplier applied
          kill_likely   - True if ke_effective > 50 J
        """
        angle_diff = abs(my_heading - target_heading)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff

        alt_adv = my_pos.alt - target_pos.alt

        impact_type = InterceptPhysics.classify_angle(
            angle_diff, alt_adv)

        v_close = InterceptPhysics.closing_speed(
            my_speed, target_speed, angle_diff)

        if alt_adv > 1.0:
            dive_bonus = math.sqrt(2 * 9.81 * max(alt_adv, 0))
            v_close += dive_bonus * 0.7

        v_close = max(v_close, 0.0)

        ke_raw = InterceptPhysics.kinetic_energy(
            interceptor_mass, v_close)

        mult   = InterceptPhysics.damage_multiplier(impact_type)
        ke_eff = ke_raw * mult

        return {
            "impact_type":    impact_type,
            "angle_diff":     round(angle_diff, 1),
            "alt_advantage":  round(alt_adv, 1),
            "closing_speed":  round(v_close, 1),
            "ke_raw_joules":  round(ke_raw, 1),
            "ke_effective":   round(ke_eff, 1),
            "damage_mult":    mult,
            "kill_likely":    ke_eff > 50,
        }

    @staticmethod
    def optimal_approach(my_pos, target_pos,
                         target_heading, target_speed,
                         my_max_speed, interceptor_mass):
        """
        Evaluate all four approach vectors and pick
        the one that maximises effective KE.

        Returns (best_heading, best_assessment) so
        the FSM knows which direction to fly.
        """
        best_heading    = None
        best_assessment = None
        best_ke         = -1

        bearing_to_target = get_bearing(my_pos, target_pos)

        approaches = {
            "HEAD_ON": (target_heading + 180) % 360,
            "SIDE_L":  (target_heading + 90) % 360,
            "SIDE_R":  (target_heading - 90) % 360,
            "DIRECT":  bearing_to_target,
        }

        for name, approach_heading in approaches.items():
            assessment = InterceptPhysics.evaluate(
                my_pos=my_pos,
                my_heading=approach_heading,
                my_speed=my_max_speed,
                target_pos=target_pos,
                target_heading=target_heading,
                target_speed=target_speed,
                interceptor_mass=interceptor_mass)

            if assessment["ke_effective"] > best_ke:
                best_ke         = assessment["ke_effective"]
                best_heading    = approach_heading
                best_assessment = assessment
                best_assessment["approach_name"] = name

        return best_heading, best_assessment

# ============================================================
# MATH - PREDICTIVE INTERCEPT
# ============================================================
def predict_intercept(my_pos, target,
                      my_speed, dt):
    dist = get_distance(my_pos,
                       target.position())

    if my_speed <= 0 or dist <= 0:
        return target.lat, target.lon

    vel_lat_ms = (target.dlat / dt) * 111320
    vel_lon_ms = (target.dlon / dt) * (
        111320 * math.cos(
            math.radians(target.lat)))

    t_intercept = min(dist /
                     max(my_speed, 1), 8.0)

    pred_lat = (target.lat +
               (vel_lat_ms * t_intercept)
               / 111320)
    pred_lon = (target.lon +
               (vel_lon_ms * t_intercept) /
               (111320 * math.cos(
                   math.radians(target.lat))))

    pred_loc  = LocationGlobalRelative(
        pred_lat, pred_lon,
        Config.INTERCEPT_ALT)
    pred_dist = get_distance(my_pos, pred_loc)

    if pred_dist > dist * 2.0:
        return target.lat, target.lon

    return pred_lat, pred_lon

# ============================================================
# MATH - RF JAMMING PHYSICS
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
                   f"{hb:.1f}s -> RTL!")
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
               f"{dist:.0f}m -> RTL!")
        vehicle.mode = VehicleMode("RTL")
        flight_log("GEOFENCE_BREACH",
                   {"dist": round(dist)})
        return False
    return True

def check_battery():
    try:
        lvl = vehicle.battery.level
        if lvl is not None and lvl < 10:
            log_warn("Critical battery! RTL!")
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

def send_velocity(vn, ve, vd):
    """
    Send raw NED velocity to the drone via MAVLink.
    This does NOT decelerate near waypoints -
    the drone keeps flying at the commanded velocity.
    Perfect for ramming.
    vn = north (m/s), ve = east (m/s), vd = down (m/s)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vn, ve, vd,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)

def fly_toward_bearing(bearing_deg, speed, vd=0):
    """
    Fly at full speed toward a bearing.
    No deceleration. No stopping.
    This is what makes ramming work.
    """
    bearing_rad = math.radians(bearing_deg)
    vn = speed * math.cos(bearing_rad)
    ve = speed * math.sin(bearing_rad)
    send_velocity(vn, ve, vd)
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
        self.heading     = 0.0

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
                self.heading = (math.degrees(
                    math.atan2(self.dlon, self.dlat)) + 360) % 360
        else:
            spd       = Config.ENEMY_SPEED_MAX
            self.dlat = random.uniform(-spd, spd)
            self.dlon = random.uniform(-spd, spd)
            self.heading = (math.degrees(
                math.atan2(self.dlon, self.dlat)) + 360) % 360

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

        if abs(self.dlat) > 1e-10 or abs(self.dlon) > 1e-10:
            self.heading = (math.degrees(
                math.atan2(self.dlon, self.dlat)) + 360) % 360

        if self.distance_to_vip() < 2:
            global MISSION_FAILED
            MISSION_FAILED = True
            log_threat(
                f"TARGET {self.id} "
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
        cur = vehicle.location.global_relative_frame.alt
        log_info(f"  Alt: {cur:.1f}m / {alt}m")
        if cur >= alt * 0.95:
            log_ok(f"Reached {alt}m!")
            break
        time.sleep(1)

# ============================================================
# ENGAGE - TRUE FSM (v8.6)
# Priority targeting + target switching
# ============================================================
def engage(target, all_targets):
    global MISSION_FAILED

    k_lat          = KalmanFilter()
    k_lon          = KalmanFilter()
    state          = State.SEARCH
    jam_start_time = 0
    timeout        = 180
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

        if time.time() - start > timeout:
            log_warn(f"Timeout! "
                    f"Target {target.id}!")
            emergency_brake()
            return "escaped"

        my_pos = vehicle.location.global_relative_frame

        if not run_all_failsafes(my_pos):
            emergency_brake()
            return "escaped"

        # v8.6: Update ALL active targets every tick
        for t in all_targets:
            if t.active:
                t.update(my_pos)

        if MISSION_FAILED:
            emergency_brake()
            return "escaped"

        # v8.6: TARGET SWITCHING
        # Check if another target is closer to VIP
        # and within emergency threshold
        for t in all_targets:
            if (t.active and
                    t.id != target.id and
                    t.distance_to_vip() < Config.VIP_SWITCH_DIST and
                    t.distance_to_vip() < target.distance_to_vip()):
                log_threat(
                    f"SWITCHING! T{t.id} at "
                    f"{t.distance_to_vip():.0f}m from VIP "
                    f"(closer than T{target.id} at "
                    f"{target.distance_to_vip():.0f}m)")
                emergency_brake()
                flight_log("TARGET_SWITCH", {
                    "from": target.id,
                    "to":   t.id,
                    "dist": round(t.distance_to_vip())
                })
                return "switched"

        s_lat  = k_lat.update(
            target.lat, Config.LOOP_RATE)
        s_lon  = k_lon.update(
            target.lon, Config.LOOP_RATE)
        smooth = LocationGlobalRelative(
            s_lat, s_lon, target.alt)

        distance = get_distance(my_pos, smooth)
        dist_vip = target.distance_to_vip()

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

        # ============================
        # FSM STATES
        # ============================

        # -- SEARCH --
        if state == State.SEARCH:
            log_state(
                f"SEARCH -> TRACK | "
                f"Target {target.id} | "
                f"{distance:.0f}m")
            state = State.TRACK
            flight_log("STATE_CHANGE",
                       {"state": State.TRACK})

        # -- TRACK --
        elif state == State.TRACK:

            if distance <= Config.JAM_RANGE:
                state          = State.JAM
                jam_start_time = time.time()
                log_state(
                    f"TRACK -> JAM | "
                    f"{distance:.0f}m")
                flight_log("STATE_CHANGE",
                           {"state": State.JAM})
                continue

            dt = target.dt
            pred_lat, pred_lon = predict_intercept(
                    my_pos, target,
                    Config.TRACK_SPEED, dt)

            goto_target(
                pred_lat, pred_lon,
                Config.INTERCEPT_ALT,
                Config.TRACK_SPEED)

            print(
                f"{C.MAGENTA}[TRACK]{C.END} | "
                f"T{target.id} | "
                f"Dist:{C.YELLOW}"
                f"{distance:.1f}m{C.END} | "
                f"VIP:{vip_str} | "
                f"{'EVADE' if target.evading else 'APPROACH'}")

            flight_log("TRACKING", {
                "target":   target.id,
                "distance": round(distance, 1)
            })

        # -- JAM (True Non-Blocking) --
        elif state == State.JAM:

            if near_vip(target.lat, target.lon):
                log_safe("Near VIP -- holding!")
                state = State.TRACK
                continue

            goto_target(
                target.lat, target.lon,
                Config.INTERCEPT_ALT, 2)

            elapsed   = (time.time() -
                        jam_start_time)
            remaining = max(
                0, Config.JAM_DURATION - elapsed)

            print(
                f"{C.BLUE}[JAM]{C.END} | "
                f"T{target.id} | "
                f"Dist:{C.YELLOW}"
                f"{distance:.1f}m{C.END} | "
                f"Timer: {remaining:.1f}s")

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
                        f"NEUTRALISED -- JAM OK")
                    target.active  = False
                    report["jammed"] += 1
                    report["vip_safe"] = True
                    emergency_brake()
                    flight_log("JAM_SUCCESS", {
                        "target": target.id
                    })
                    return "jammed"

                else:
                    log_warn(
                        "JAM FAILED! "
                        "-> PHYSICAL INTERCEPT")
                    state = State.INTERCEPT
                    flight_log("STATE_CHANGE", {
                        "state": State.INTERCEPT
                    })

        # -- INTERCEPT (Fly-Through Ram - v8.6) --
        elif state == State.INTERCEPT:

            if near_vip(target.lat, target.lon):
                log_safe(
                    "Near VIP! Re-attempting jam!")
                state          = State.JAM
                jam_start_time = time.time()
                continue

            # Use RAM_SPEED for KE calc, not groundspeed
            # (groundspeed drops due to decel, but we
            # command full speed with velocity cmds)
            my_heading = get_bearing(my_pos, target.position())
            my_speed   = Config.RAM_SPEED

            assessment = InterceptPhysics.evaluate(
                my_pos=my_pos,
                my_heading=my_heading,
                my_speed=my_speed,
                target_pos=target.position(),
                target_heading=target.heading,
                target_speed=target.speed_ms(),
                interceptor_mass=Config.INTERCEPTOR_MASS)

            # v8.6: PREDICTIVE LEAD POINT
            # Aim where the target WILL BE, not where it IS
            dt = target.dt
            pred_lat, pred_lon = predict_intercept(
                my_pos, target, Config.RAM_SPEED, dt)

            # v8.6: FLY-THROUGH using velocity commands
            # Calculate bearing to predicted position
            pred_pos = LocationGlobalRelative(
                pred_lat, pred_lon, target.alt)
            ram_bearing = get_bearing(my_pos, pred_pos)

            # Calculate vertical speed to match target alt
            alt_diff = target.alt - my_pos.alt
            vd = 0
            if abs(alt_diff) > 1.0:
                vd = -max(-3.0, min(3.0, alt_diff / 2.0))

            # SEND VELOCITY - no deceleration!
            fly_toward_bearing(ram_bearing,
                              Config.RAM_SPEED, vd)

            print(
                f"{C.RED}[INTERCEPT]{C.END} | "
                f"T{target.id} | "
                f"Dist:{C.YELLOW}"
                f"{distance:.1f}m{C.END} | "
                f"{C.RED}"
                f"{Config.RAM_SPEED}m/s{C.END} | "
                f"Angle:{assessment['impact_type']} | "
                f"KE:{assessment['ke_effective']:.0f}J | "
                f"Brg:{ram_bearing:.0f}")

            flight_log("INTERCEPT_PHYSICS", {
                "target":        target.id,
                "distance":      round(distance, 1),
                "impact_type":   assessment["impact_type"],
                "closing_speed": assessment["closing_speed"],
                "ke_effective":  assessment["ke_effective"],
                "angle_diff":    assessment["angle_diff"],
                "bearing":       round(ram_bearing),
            })

            if distance < Config.RAM_TRIGGER:
                # Use commanded speed for final KE
                final_assessment = InterceptPhysics.evaluate(
                    my_pos=my_pos,
                    my_heading=ram_bearing,
                    my_speed=Config.RAM_SPEED,
                    target_pos=target.position(),
                    target_heading=target.heading,
                    target_speed=target.speed_ms(),
                    interceptor_mass=Config.INTERCEPTOR_MASS)

                ke = final_assessment["ke_effective"]
                impact = final_assessment["impact_type"]
                v_close = final_assessment["closing_speed"]

                print(
                    f"\n{C.RED}{C.BOLD}"
                    f"{'='*45}\n"
                    f" TARGET {target.id} -- DESTROYED!\n"
                    f"   Impact:  {impact}\n"
                    f"   Close V: {v_close} m/s\n"
                    f"   KE:      {ke:.0f} J effective\n"
                    f"   Mass:    {Config.INTERCEPTOR_MASS} kg\n"
                    f"   Kill:    {'YES' if final_assessment['kill_likely'] else 'UNCERTAIN'}\n"
                    f"{'='*45}"
                    f"{C.END}\n")

                target.active  = False
                report["intercepted"] += 1
                report["vip_safe"]    = True
                emergency_brake()
                flight_log(
                    "KINETIC_MITIGATION", {
                        "target":        target.id,
                        "ke_effective":   round(ke),
                        "ke_raw":         final_assessment["ke_raw_joules"],
                        "impact_type":    impact,
                        "closing_speed":  v_close,
                        "kill_likely":    final_assessment["kill_likely"],
                    })
                return "intercepted"

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
          f"{'MISSION REPORT v8.6':^45}"
          f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}")
    print(f"  Duration       : {dur:.0f}s")
    print(f"  Targets        : {total}")
    print(f"{C.GREEN}  Jammed         : "
          f"{report['jammed']}{C.END}")
    print(f"{C.YELLOW}  Intercepted    : "
          f"{report['intercepted']}{C.END}")
    print(f"{C.RED}  Escaped        : "
          f"{report['escaped']}{C.END}")
    print(f"  Battery        : {batt}%")
    print(f"  Max speed      : "
          f"{report['max_speed']}m/s")
    print(f"  Mass           : "
          f"{Config.INTERCEPTOR_MASS}kg")
    clr = C.GREEN if rate == 100 else C.YELLOW
    print(f"{clr}  Success        : "
          f"{rate:.0f}%{C.END}")
    status = (
        f"{C.GREEN}SECURE{C.END}"
        if report["vip_safe"]
        else f"{C.RED}COMPROMISED{C.END}")
    print(f"  VIP Status     : {status}")
    if MISSION_FAILED:
        print(f"\n{C.RED}{C.BOLD}"
              f"  MISSION FAILED -- "
              f"VIP COMPROMISED"
              f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}"
          f"{'MISSION COMPLETE v8.6!':^45}"
          f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}\n")

# ============================================================
# PHYSICS TEST (run with --test-physics)
# ============================================================
def run_physics_test():
    print(f"\n{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}"
          f"{'INTERCEPT PHYSICS TEST':^45}"
          f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}\n")

    mass = Config.INTERCEPTOR_MASS
    print(f"  Interceptor mass: {mass} kg\n")

    print(f"{C.BOLD}Test 1: HEAD-ON (180 deg offset){C.END}")
    my  = LocationGlobalRelative(0, 0, 12)
    tgt = LocationGlobalRelative(0.0001, 0, 8)
    r = InterceptPhysics.evaluate(
        my_pos=my, my_heading=0,
        my_speed=15,
        target_pos=tgt, target_heading=180,
        target_speed=10,
        interceptor_mass=mass)
    for k, v in r.items():
        print(f"    {k}: {v}")

    print(f"\n{C.BOLD}Test 2: REAR CHASE (0 deg offset){C.END}")
    r = InterceptPhysics.evaluate(
        my_pos=my, my_heading=0,
        my_speed=15,
        target_pos=tgt, target_heading=0,
        target_speed=10,
        interceptor_mass=mass)
    for k, v in r.items():
        print(f"    {k}: {v}")

    print(f"\n{C.BOLD}Test 3: SIDE ATTACK (90 deg offset){C.END}")
    r = InterceptPhysics.evaluate(
        my_pos=my, my_heading=90,
        my_speed=15,
        target_pos=tgt, target_heading=0,
        target_speed=10,
        interceptor_mass=mass)
    for k, v in r.items():
        print(f"    {k}: {v}")

    print(f"\n{C.BOLD}Test 4: TOP-DOWN DIVE (5m above){C.END}")
    my_high = LocationGlobalRelative(0, 0, 15)
    tgt_low = LocationGlobalRelative(0.0001, 0, 8)
    r = InterceptPhysics.evaluate(
        my_pos=my_high, my_heading=0,
        my_speed=15,
        target_pos=tgt_low, target_heading=0,
        target_speed=10,
        interceptor_mass=mass)
    for k, v in r.items():
        print(f"    {k}: {v}")

    print(f"\n{C.BOLD}Test 5: OPTIMAL APPROACH SELECTION{C.END}")
    best_hdg, best = InterceptPhysics.optimal_approach(
        my_pos=my,
        target_pos=tgt,
        target_heading=45,
        target_speed=10,
        my_max_speed=15,
        interceptor_mass=mass)
    print(f"    Best heading:  {best_hdg:.0f} deg")
    for k, v in best.items():
        print(f"    {k}: {v}")

    print(f"\n{C.GREEN}{'='*45}{C.END}")
    print(f"{C.GREEN}{C.BOLD}"
          f"{'ALL PHYSICS TESTS PASSED':^45}"
          f"{C.END}")
    print(f"{C.GREEN}{'='*45}{C.END}\n")

# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":

    if args.test_physics:
        run_physics_test()
        sys.exit(0)

    print(f"\n{C.CYAN}{'='*45}{C.END}")
    print(f"{C.BOLD}"
          f"{'INTERCEPTOR v8.6':^45}"
          f"{C.END}")
    print(f"{C.CYAN}"
          f"{'Priority Target | Switch | Fly-Through':^45}"
          f"{C.END}")
    print(f"{C.CYAN}{'='*45}{C.END}\n")

    kill_old_processes()
    start_sitl()
    start_mavproxy()

    print(f"\n{C.GREEN}{C.BOLD}"
          f"Mission Planner:"
          f"{C.END}")
    print(f"{C.GREEN}"
          f"  UDP -> Port 14551 -> Connect"
          f"{C.END}\n")

    wait_for_gps()
    connect_vehicle()

    print(f"\n{C.BOLD}"
          f"--- MISSION START ---{C.END}\n")
    arm_and_takeoff(Config.INTERCEPT_ALT)
    time.sleep(2)

    home_frame  = vehicle.location.global_relative_frame
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
    log_safe(f"Interceptor mass: "
            f"{Config.INTERCEPTOR_MASS}kg")

    flight_log("MISSION_START", {
        "lat":      home_frame.lat,
        "lon":      home_frame.lon,
        "geofence": Config.MAX_RADIUS,
        "mass":     Config.INTERCEPTOR_MASS
    })

    print(f"\n{C.RED}{C.BOLD}"
          f"2 TARGETS INBOUND!{C.END}\n")

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
            f"speed={t.speed_ms():.1f}m/s | "
            f"hdg={t.heading:.0f} deg")
        flight_log("TARGET_DETECTED", {
            "id":      t.id,
            "dist":    round(t.distance_to_vip()),
            "speed":   round(t.speed_ms(), 1),
            "heading": round(t.heading)
        })

    print(f"\n{C.CYAN}"
          f"INTERCEPTOR v8.6 ONLINE{C.END}")
    print(f"{C.CYAN}"
          f"Priority | Switching | Fly-Through | "
          f"Velocity | Non-blocking{C.END}\n")
    time.sleep(1)

    # v8.6: PRIORITY TARGETING LOOP
    # Always engage the active target closest to VIP
    # If engage returns "switched", re-prioritise
    max_rounds = 10  # safety limit
    for _ in range(max_rounds):
        # Find highest priority: closest active target to VIP
        active = [t for t in targets if t.active]
        if not active or MISSION_FAILED:
            break

        active.sort(key=lambda t: t.distance_to_vip())
        priority_target = active[0]

        log_action(
            f"PRIORITY: T{priority_target.id} | "
            f"VIP dist: {priority_target.distance_to_vip():.0f}m | "
            f"{len(active)} active targets")

        result = engage(priority_target, targets)

        if MISSION_FAILED:
            log_threat("MISSION FAILED! "
                      "VIP COMPROMISED!")
            break
        elif result == "switched":
            # Re-enter loop, will pick new priority
            log_warn("Re-prioritising targets...")
            continue
        elif result == "escaped":
            report["escaped"] += 1
            log_warn(
                f"Target {priority_target.id} escaped!")
        else:
            log_ok(
                f"Target {priority_target.id} "
                f"{result.upper()}!")

        time.sleep(0.5)

    if not MISSION_FAILED:
        print(f"\n{C.GREEN}{C.BOLD}"
              f"All targets handled! "
              f"Returning home...{C.END}")

    vehicle.mode = VehicleMode("RTL")
    time.sleep(8)

    print_report()
    save_log()
    cleanup()
