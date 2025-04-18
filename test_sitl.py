import time
from pymavlink import mavutil

# ========================
# CONNECT TO SITL
# ========================
print("Connecting to SITL (UDP 127.0.0.1:14550)...")
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")

# Wait for the first heartbeat to ensure connection
master.wait_heartbeat()
print(f"Heartbeat received from system ({master.target_system}, {master.target_component})")

# ========================
# FUNCTION: CHANGE MODE
# ========================
def set_mode(mode):
    """Sets flight mode (e.g., GUIDED, AUTO, LAND)."""
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode}")

# =========================
# FUNCTION: Move in a DIRECTION
# ========================

    #Lowkey just shows velocity rofl

def send_directional_velocity(forward, right, down, duration=2):
    print(f"Moving: forward={forward}, right={right}, down={down}")
    end_time = time.time() + duration
    while time.time() < end_time:
        master.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10,  # time_boot_ms (arbitrary)
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # Only velocity components enabled
                0, 0, 0,  # position (not used)
                forward, right, down,  # velocity in m/s
                0, 0, 0,  # acceleration (not used)
                0, 0  # yaw, yaw_rate
            )
        )
        time.sleep(0.1)  # Small delay to prevent message spam

# =======================
# FUNCTION: MOVES BASED ON A QUANDRANT
# =======================

def move_based_on_quadrant(quadrant):
    speed = 1.0  # Adjust speed (m/s)
    if quadrant == "Q1":      # Forward + Right
        send_directional_velocity(speed, speed, 0)
    elif quadrant == "Q2":    # Forward + Left
        send_directional_velocity(speed, -speed, 0)
    elif quadrant == "Q3":    # Backward + Left
        send_directional_velocity(-speed, -speed, 0)
    elif quadrant == "Q4":    # Backward + Right
        send_directional_velocity(-speed, speed, 0)
    else:
        print("Unknown quadrant; hovering.")


# ========================
# FUNCTION: ARM DRONE
# ========================
def arm_drone():
    """Arms the drone."""
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Drone Armed")

# ========================
# FUNCTION: TAKE OFF
# ========================
def takeoff(altitude):
    """Commands the drone to take off to the given altitude (meters)."""
    print(f"Taking off to {altitude}m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Confirmation
        0, 0, 0, 0,  # Params (unused)
        0, 0, altitude  # x, y, altitude
    )
    time.sleep(10)  # Wait for takeoff to complete

# ========================
# FUNCTION: FLY TO WAYPOINT
# ========================
def goto_position(lat, lon, alt):
    """Commands the drone to fly to a specific GPS coordinate."""
    print(f"Flying to ({lat}, {lon}) at {alt}m...")
    master.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            10,  # Time delay (ms)
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b110111111000),  # Control mask (only position enabled)
            int(lat * 1e7),  # Latitude in degrees * 1E7
            int(lon * 1e7),  # Longitude in degrees * 1E7
            alt,  # Altitude in meters
            0, 0, 0,  # Velocity (not used)
            0, 0, 0,  # Acceleration (not used)
            0,  # Yaw
            0  # Yaw rate
        )
    )
    time.sleep(10)  # Wait to reach waypoint

# ========================
# FUNCTION: LAND
# ========================
def land_drone():
    """Commands the drone to land."""
    print("Landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # Confirmation
        0, 0, 0, 0,  # Params (unused)
        0, 0, 0  # x, y, altitude
    )
    time.sleep(10)  # Allow time to land

# ========================
# MAIN SEQUENCE
# ========================
if __name__ == "__main__":
    set_mode("GUIDED")
    arm_drone()
    takeoff(10)

    print("Starting quadrant-based movement loop... Press Ctrl+C to stop.")
    try:
        while True:
            try:
                with open("quadrant.txt", "r") as f:
                    quadrant = f.read().strip()
                print(f"Detected Quadrant: {quadrant}")
                move_based_on_quadrant(quadrant)
            except FileNotFoundError:
                print("quadrant.txt not found, waiting...")
            except Exception as e:
                print(f"Error reading quadrant.txt: {e}")

            time.sleep(1)  # Check every 1 second

    except KeyboardInterrupt:
        print("Interrupted by user. Landing drone...")
        land_drone()

