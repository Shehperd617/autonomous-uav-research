#!/usr/bin/env python3
import time
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- CONSTANTS & FAILSAFES ---
MAX_FLIGHT_RADIUS_M = 2000.0  # Geofence boundary
INTERCEPT_RADIUS_M = 15.0     # Strike distance
N_CONST = 4.0                 # ProNav Navigation Constant
MIN_ALT_M = 10.0              # Hard deck altitude failsafe
CRIT_VOLTAGE = 14.0           # Failsafe voltage (set for 4S in SITL, change to 21.0 for 6S IRL)

def get_distance_meters(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

def get_local_xy(home_loc, curr_loc):
    x = (curr_loc.lon - home_loc.lon) * 111319.5 * math.cos(math.radians(home_loc.lat))
    y = (curr_loc.lat - home_loc.lat) * 111319.5
    return x, y

def set_attitude_thrust(vehicle, roll_deg, pitch_deg, yaw_deg, thrust):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr2 = math.cos(roll * 0.5)
    cp2 = math.cos(pitch * 0.5)
    cy2 = math.cos(yaw * 0.5)
    sr2 = math.sin(roll * 0.5)
    sp2 = math.sin(pitch * 0.5)
    sy2 = math.sin(yaw * 0.5)

    q = [
        cr2 * cp2 * cy2 + sr2 * sp2 * sy2,
        sr2 * cp2 * cy2 - cr2 * sp2 * sy2,
        cr2 * sp2 * cy2 + sr2 * cp2 * sy2,
        cr2 * cp2 * sy2 - sr2 * sp2 * cy2
    ]

    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 1, 1, 7, q, 0, 0, 0, thrust
    )
    vehicle.send_mavlink(msg)

def arm_and_takeoff(vehicle, aTargetAltitude):
    print("  Waiting for armable ...")
    while not vehicle.is_armable:
        time.sleep(1)
    print("  Armable.")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    print("  Armed! Taking off ...")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= aTargetAltitude * 0.95:
            print(f"  Alt {alt:.1f} m -- takeoff done.\n")
            break
        time.sleep(1)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='tcp:127.0.0.1:5760')
    args = parser.parse_args()

    print(f"Connecting to {args.connect} ...\n")
    vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=60)

    arm_and_takeoff(vehicle, 60.0)
    home_loc = vehicle.location.global_relative_frame

    print("==================================================")
    print("  THE SPEAR v12.4 — PRONAV KINETIC INTERCEPTOR")
    print("  Military-Grade Tracking & Failsafes Active")
    print("==================================================")

    targets = [
        {"id": 1, "lat": home_loc.lat + 0.002, "lon": home_loc.lon + 0.002, "alt": 30},
        {"id": 2, "lat": home_loc.lat + 0.004, "lon": home_loc.lon - 0.003, "alt": 30},
    ]

    path_x, path_y, path_z = [], [], []
    peak_gs = 0.0
    start_time = time.time()

    for t in targets:
        target_loc = LocationGlobalRelative(t["lat"], t["lon"], t["alt"])
        intercepted = False
        
        # Reset ProNav state for the new target
        prev_bearing = None
        prev_dist = None
        prev_time = time.time()
        
        while not intercepted:
            curr_loc = vehicle.location.global_relative_frame
            current_time = time.time()
            dt = current_time - prev_time
            
            if dt <= 0.0:
                time.sleep(0.01)
                continue

            dist = get_distance_meters(curr_loc, target_loc)
            bearing = get_bearing(curr_loc, target_loc)
            gs = vehicle.groundspeed
            
            x, y = get_local_xy(home_loc, curr_loc)
            path_x.append(x)
            path_y.append(y)
            path_z.append(curr_loc.alt)

            if gs > peak_gs:
                peak_gs = gs

            # --- 1. STRICT SAFETY FAILSAFES ---
            if get_distance_meters(home_loc, curr_loc) > MAX_FLIGHT_RADIUS_M:
                print(f"\n[FAILSAFE] Geofence Breach ({MAX_FLIGHT_RADIUS_M}m) -> ABORT STRIKE!")
                vehicle.mode = VehicleMode("RTL")
                return

            if curr_loc.alt < MIN_ALT_M:
                print(f"\n[FAILSAFE] Hard Deck Breach ({curr_loc.alt:.1f}m) -> CLIMBING!")
                set_attitude_thrust(vehicle, roll_deg=0.0, pitch_deg=-10.0, yaw_deg=bearing, thrust=1.0)
                time.sleep(0.5)
                continue
                
            if vehicle.battery.voltage is not None and vehicle.battery.voltage > 0:
                if vehicle.battery.voltage < CRIT_VOLTAGE:
                    print(f"\n[FAILSAFE] Voltage Sag ({vehicle.battery.voltage}V) -> ABORT STRIKE!")
                    vehicle.mode = VehicleMode("RTL")
                    return

            # --- 2. PRONAV MATH (LAMBDA DOT) ---
            if prev_bearing is not None and prev_dist is not None:
                closing_speed = (prev_dist - dist) / dt
                
                delta_bearing = bearing - prev_bearing
                if delta_bearing > 180.0: delta_bearing -= 360.0
                if delta_bearing < -180.0: delta_bearing += 360.0
                los_rate = delta_bearing / dt
                
                # a_c = N * V_c * lambda_dot
                lead_angle = N_CONST * los_rate * max(1.0, (closing_speed / 10.0))
                lead_angle = max(-45.0, min(45.0, lead_angle))
                intercept_heading = (bearing + lead_angle) % 360.0
            else:
                intercept_heading = bearing
                closing_speed = 0.0

            prev_bearing = bearing
            prev_dist = dist
            prev_time = current_time

            # --- 3. PROXIMITY FUSE ---
            if dist < INTERCEPT_RADIUS_M or (closing_speed > 20.0 and dist < 25.0):
                mission_t = time.time() - start_time
                print(f"\n  *** TARGET KINETIC INTERCEPT d={dist:.1f}m GS={gs:.1f}m/s t={mission_t:.1f}s ***\n")
                set_attitude_thrust(vehicle, 0.0, 45.0, intercept_heading, 1.0)
                time.sleep(1)
                intercepted = True
                break

            # --- 4. FLIGHT CONTROL EXECUTION ---
            yaw_error = intercept_heading - vehicle.heading
            yaw_error = (yaw_error + 180) % 360 - 180
            attack_roll = max(-45.0, min(45.0, yaw_error * 1.5))

            alt_error = curr_loc.alt - target_loc.alt
            if alt_error > 20.0:
                attack_pitch = -60.0  
            else:
                attack_pitch = -40.0  

            set_attitude_thrust(vehicle, roll_deg=attack_roll, pitch_deg=attack_pitch, yaw_deg=intercept_heading, thrust=1.0)
            
            print(f"  T#{t['id']} [PRONAV] d={dist:5.1f}m GS={gs:4.1f} Vc={closing_speed:4.1f} | hdg={intercept_heading:5.1f} roll={attack_roll:4.1f}")
            time.sleep(0.05) # 20Hz control loop

    print("==================================================")
    print(f"  MISSION COMPLETE -- {time.time() - start_time:.1f}s")
    print(f"  Peak groundspeed: {peak_gs:.1f} m/s")
    print("==================================================")
    
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()

    print("\nGenerating 3D Flight Trajectory...")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(path_x, path_y, path_z, label='Spear Trajectory', color='cyan', linewidth=2)
    ax.scatter([0], [0], [60], color='blue', marker='^', s=150, label='Launch / Home')
    
    for t in targets:
        t_loc = LocationGlobalRelative(t["lat"], t["lon"], t["alt"])
        tx, ty = get_local_xy(home_loc, t_loc)
        ax.scatter([tx], [ty], [t["alt"]], color='red', marker='X', s=150, label=f'Target {t["id"]}')

    ax.set_xlabel('X (Meters East)')
    ax.set_ylabel('Y (Meters North)')
    ax.set_zlabel('Altitude (Meters)')
    ax.set_title('The Spear: ProNav Kinetic Intercept')
    ax.legend()
    plt.savefig("intercept_trajectory.png", dpi=300)
    print("Graph saved to 'intercept_trajectory.png'.")

if __name__ == '__main__':
    main()
