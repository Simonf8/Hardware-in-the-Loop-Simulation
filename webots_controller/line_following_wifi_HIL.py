# line_following_wifi_HIL.py (Webots Controller - TCP Client)

from controller import Robot
import socket
import time
from math import cos, sin

# --- Network Configuration ---
# !!! USER ACTION REQUIRED: Update this with your ESP32's IP address !!!
ESP32_IP_ADDRESS = "192.168.53.193"  # <<<<<<<<<<< CHANGE THIS
ESP32_PORT = 8266
# ---------------------------

# --- Robot Configuration (same as smoother serial version) ---
LINE_THRESHOLD = 600
MAX_FORWARD_SPEED = 1.5
GENTLE_TURN_OUTER_SPEED = 1.2
GENTLE_TURN_INNER_SPEED = 0.5
SHARP_TURN_ROTATE_SPEED = 1.0
SEARCH_ROTATE_SPEED = 0.75
# -------------------------------------------------------------

# 1) Initialize Robot and time step
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2) Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
if left_motor is None or right_motor is None: print("❌ Error: Motors not found."); exit(1)
for m in (left_motor, right_motor): m.setPosition(float('inf')); m.setVelocity(0.0)
print("✅ Webots: Motors initialized.")

# 3) Ground sensors
sensor_names = ['gs0', 'gs1', 'gs2']
ground_sensors = []
for name in sensor_names:
    sensor_device = robot.getDevice(name)
    if sensor_device is None: print(f"⚠️ Warning: Ground sensor '{name}' not found.")
    else: sensor_device.enable(timestep); ground_sensors.append(sensor_device)
if len(ground_sensors) != 3: print(f"❌ Error: Expected 3 ground sensors, found {len(ground_sensors)}."); exit(1)
print("✅ Webots: All 3 ground sensors initialized.")

# Initialize distance sensors (for obstacle detection)
distance_sensor_names = ['ds_left', 'ds_front', 'ds_right']
distance_sensors = []
for name in distance_sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None: 
        print(f"⚠️ Warning: Distance sensor '{name}' not found.")
    else: 
        sensor.enable(timestep)
        distance_sensors.append(sensor)
print(f"✅ Webots: {len(distance_sensors)} distance sensors initialized.")

# Initialize wheel encoders
left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
if left_encoder and right_encoder:
    left_encoder.enable(timestep)
    right_encoder.enable(timestep)
    print("✅ Webots: Wheel encoders initialized.")
else:
    print("⚠️ Warning: Wheel encoders not found.")

# Robot physical parameters for position tracking
WHEEL_RADIUS = 0.0205  # meters
AXLE_LENGTH = 0.052    # meters
position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # Robot's position and orientation
last_left_encoder = 0.0
last_right_encoder = 0.0

# 4) Network Client Setup
client_socket = None
connection_established = False

def setup_network_client():
    global client_socket, connection_established
    if client_socket: # Close existing socket if any before reconnecting
        try: client_socket.close()
        except: pass
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.settimeout(5.0) # Timeout for connection attempt
    print(f"Webots: Attempting to connect to ESP32 server at {ESP32_IP_ADDRESS}:{ESP32_PORT}...")
    try:
        client_socket.connect((ESP32_IP_ADDRESS, ESP32_PORT))
        client_socket.settimeout(1.0) # Shorter timeout for send/recv operations
        connection_established = True
        print(f"✅ Webots: Connected to ESP32 server at {ESP32_IP_ADDRESS}:{ESP32_PORT}")
    except socket.timeout:
        print(f"⚠️ Webots: Connection attempt timed out. ESP32 server not found or not responding at {ESP32_IP_ADDRESS}:{ESP32_PORT}.")
        connection_established = False
    except ConnectionRefusedError:
        print(f"⚠️ Webots: Connection refused. ESP32 server might not be listening on {ESP32_IP_ADDRESS}:{ESP32_PORT}.")
        connection_established = False
    except Exception as e:
        print(f"❌ Webots: Error connecting to ESP32 server: {e}")
        connection_established = False
    return connection_established

# Initial connection attempt
setup_network_client()

# 5) Main control loop state
current_commanded_state = 'stop'
last_successful_communication_time = time.time()

print(f"\nWebots: Robot controller started. Attempting Wi-Fi HIL with ESP32.")

while robot.step(timestep) != -1:
    if not connection_established:
        if time.time() - last_successful_communication_time > 5.0: # Retry connection every 5 seconds
            print("Webots: Attempting to reconnect to ESP32...")
            setup_network_client()
            last_successful_communication_time = time.time() # Reset retry timer
        # Keep robot stopped if not connected
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        time.sleep(0.1) # Prevent busy-looping connection retries too fast
        continue

    # a) Read sensors and process data
    # Ground sensors
    raw_sensor_values = [s.getValue() for s in ground_sensors]
    on_line_flags = [value < LINE_THRESHOLD for value in raw_sensor_values]
    
    # Distance sensors
    obstacle_distances = [s.getValue() if s else float('inf') for s in distance_sensors]
    obstacles_detected = [d < 0.1 for d in obstacle_distances]  # Detect obstacles within 10cm
    
    # Update position using encoder values
    if left_encoder and right_encoder:
        # Get current encoder values
        left_wheel_ticks = left_encoder.getValue()
        right_wheel_ticks = right_encoder.getValue()
        
        # Calculate wheel movements
        left_wheel_delta = left_wheel_ticks - last_left_encoder
        right_wheel_delta = right_wheel_ticks - last_right_encoder
        
        # Update last encoder values
        last_left_encoder = left_wheel_ticks
        last_right_encoder = right_wheel_ticks
        
        # Calculate distance traveled by each wheel
        left_distance = left_wheel_delta * WHEEL_RADIUS
        right_distance = right_wheel_delta * WHEEL_RADIUS
        
        # Calculate robot movement
        distance = (left_distance + right_distance) / 2.0
        rotation = (right_distance - left_distance) / AXLE_LENGTH
        
        # Update position
        position['theta'] += rotation
        position['x'] += distance * cos(position['theta'])
        position['y'] += distance * sin(position['theta'])

    # Create message for ESP32 including line sensors, obstacles, and position
    message_parts = [
        "".join(['1' if flag else '0' for flag in on_line_flags]),  # Line sensors
        "".join(['1' if obs else '0' for obs in obstacles_detected]),  # Obstacle sensors
        f"{position['x']:.3f},{position['y']:.3f},{position['theta']:.3f}"  # Position
    ]
    sensor_message_to_esp32 = "|".join(message_parts)

    # b) Send to ESP32
    try:
        client_socket.sendall((sensor_message_to_esp32 + '\n').encode('utf-8'))
        # print(f"DEBUG Webots sent: '{sensor_message_to_esp32}'")
    except socket.timeout:
        print("⚠️ Webots: Socket send timeout. ESP32 might be unresponsive.")
        connection_established = False # Assume connection lost
        last_successful_communication_time = time.time() # Update time to control reconnect attempts
        continue
    except Exception as e:
        print(f"❌ Webots: Error sending data to ESP32: {e}")
        connection_established = False
        last_successful_communication_time = time.time()
        continue
        
    # c) Read from ESP32
    response_from_esp32 = ""
    try:
        data_bytes = client_socket.recv(128) # Buffer size
        if not data_bytes: # ESP32 closed connection
            print("⚠️ Webots: ESP32 closed the connection.")
            connection_established = False
            last_successful_communication_time = time.time()
            continue
        response_from_esp32 = data_bytes.decode('utf-8').strip()
        if response_from_esp32:
            # print(f"DEBUG Webots received: '{response_from_esp32}'")
            last_successful_communication_time = time.time() # Update time on successful comm
        else: # Empty response can mean ESP32 sent just newline or connection issue
            pass 
            
    except socket.timeout: # Expected if ESP32 doesn't respond quickly enough
        # print("DEBUG Webots: Socket recv timeout waiting for ESP32 command.") # Can be noisy
        pass # No command received this cycle, robot will continue last valid state or timeout to stop
    except Exception as e:
        print(f"❌ Webots: Error receiving data from ESP32: {e}")
        connection_established = False
        last_successful_communication_time = time.time()
        continue

    # Safety: If no command received from ESP32 for a while (even if socket seems open but ESP32 is silent)
    if connection_established and (time.time() - last_successful_communication_time > 3.0) and current_commanded_state != 'stop':
        print("⚠️ Webots: No valid command from ESP32 recently. Forcing stop.")
        current_commanded_state = 'stop'
        # No need to set response_from_esp32 here, current_commanded_state will dictate stop

    # Update state based on valid ESP32 command
    valid_commands = ['forward', 'turn_left_gentle', 'turn_left_sharp', 
                      'turn_right_gentle', 'turn_right_sharp', 'search', 'stop']
    if response_from_esp32 and response_from_esp32 in valid_commands:
        current_commanded_state = response_from_esp32
    elif response_from_esp32: # Unrecognized command
        print(f"⚠️ Webots: Unrecognized command from ESP32: '{response_from_esp32}'. Holding state: '{current_commanded_state}'")

    # d) Map state to wheel speeds
    left_speed, right_speed = 0.0, 0.0
    if current_commanded_state == 'forward': left_speed, right_speed = MAX_FORWARD_SPEED, MAX_FORWARD_SPEED
    elif current_commanded_state == 'turn_left_gentle': left_speed, right_speed = GENTLE_TURN_INNER_SPEED, GENTLE_TURN_OUTER_SPEED
    elif current_commanded_state == 'turn_left_sharp':  left_speed, right_speed = -SHARP_TURN_ROTATE_SPEED, SHARP_TURN_ROTATE_SPEED
    elif current_commanded_state == 'turn_right_gentle': left_speed, right_speed = GENTLE_TURN_OUTER_SPEED, GENTLE_TURN_INNER_SPEED
    elif current_commanded_state == 'turn_right_sharp': left_speed, right_speed = SHARP_TURN_ROTATE_SPEED, -SHARP_TURN_ROTATE_SPEED
    elif current_commanded_state == 'search': left_speed, right_speed = SEARCH_ROTATE_SPEED, -SEARCH_ROTATE_SPEED
    elif current_commanded_state == 'stop': left_speed, right_speed = 0.0, 0.0
    else:
        print(f"🚨 Webots: Unknown state '{current_commanded_state}'. Stopping.")
        left_speed, right_speed = 0.0, 0.0
        current_commanded_state = 'stop'

    # e) Apply speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    # print(f"Webots state: {current_commanded_state}, Speeds: L={left_speed:.2f}, R={right_speed:.2f}")

# Cleanup
if client_socket:
    client_socket.close()
    print("ℹ️ Webots: Client socket closed.")
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
print("ℹ️ Webots: Controller finished.")