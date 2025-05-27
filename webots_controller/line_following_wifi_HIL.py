# line_following_wifi_HIL.py (Fixed Webots Controller)

from controller import Robot
import socket
import time

# --- Network Configuration ---
ESP32_IP_ADDRESS = "192.168.53.193"  # Update with your ESP32's IP
ESP32_PORT = 8266

# --- Robot Configuration ---
LINE_THRESHOLD = 600
MAX_FORWARD_SPEED = 1.5
GENTLE_TURN_OUTER_SPEED = 1.2
GENTLE_TURN_INNER_SPEED = 0.5
SHARP_TURN_ROTATE_SPEED = 1.0
SEARCH_ROTATE_SPEED = 0.75

# Initialize Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
if left_motor is None or right_motor is None: 
    print("❌ Error: Motors not found.")
    exit(1)

for motor in (left_motor, right_motor): 
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
print("✅ Motors initialized.")

# Initialize Ground Sensors
sensor_names = ['gs0', 'gs1', 'gs2']
ground_sensors = []
for name in sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None: 
        print(f"⚠️ Warning: Ground sensor '{name}' not found.")
    else: 
        sensor.enable(timestep)
        ground_sensors.append(sensor)

if len(ground_sensors) != 3: 
    print(f"❌ Error: Expected 3 ground sensors, found {len(ground_sensors)}.")
    exit(1)
print("✅ Ground sensors initialized.")

# Initialize Distance Sensors (optional - for obstacle detection)
distance_sensor_names = ['ds_left', 'ds_front', 'ds_right']
distance_sensors = []
for name in distance_sensor_names:
    sensor = robot.getDevice(name)
    if sensor is not None:
        sensor.enable(timestep)
        distance_sensors.append(sensor)
    else:
        distance_sensors.append(None)

print(f"✅ Distance sensors initialized ({len([s for s in distance_sensors if s])}/3).")

# Network Setup
client_socket = None
connection_established = False

def setup_network_client():
    global client_socket, connection_established
    if client_socket:
        try: 
            client_socket.close()
        except: 
            pass
    
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.settimeout(5.0)
    
    print(f"Attempting to connect to ESP32 at {ESP32_IP_ADDRESS}:{ESP32_PORT}...")
    try:
        client_socket.connect((ESP32_IP_ADDRESS, ESP32_PORT))
        client_socket.settimeout(1.0)
        connection_established = True
        print(f"✅ Connected to ESP32!")
        return True
    except socket.timeout:
        print(f"⚠️ Connection timeout. Check ESP32 IP address.")
        connection_established = False
        return False
    except ConnectionRefusedError:
        print(f"⚠️ Connection refused. ESP32 server may not be running.")
        connection_established = False
        return False
    except Exception as e:
        print(f"❌ Connection error: {e}")
        connection_established = False
        return False

# Initial connection
setup_network_client()

# Control variables
current_state = 'stop'
last_comm_time = time.time()
reconnect_interval = 5.0

print("Starting robot control loop...")

# Main Control Loop
while robot.step(timestep) != -1:
    current_time = time.time()
    
    # Handle connection
    if not connection_established:
        if current_time - last_comm_time > reconnect_interval:
            print("Attempting to reconnect...")
            setup_network_client()
            last_comm_time = current_time
        
        # Keep robot stopped while disconnected
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        continue
    
    # Read ground sensors
    sensor_values = [sensor.getValue() for sensor in ground_sensors]
    line_detected = [value < LINE_THRESHOLD for value in sensor_values]
    
    # Read distance sensors (for obstacle detection)
    obstacle_distances = []
    for sensor in distance_sensors:
        if sensor:
            obstacle_distances.append(sensor.getValue())
        else:
            obstacle_distances.append(float('inf'))
    
    obstacles_detected = [d < 0.1 for d in obstacle_distances]  # 10cm threshold
    
    # Create sensor message for ESP32
    line_data = "".join(['1' if detected else '0' for detected in line_detected])
    obstacle_data = "".join(['1' if detected else '0' for detected in obstacles_detected])
    position_data = "0.0,0.0,0.0"  # Simplified - you can add real position tracking
    
    sensor_message = f"{line_data}|{obstacle_data}|{position_data}"
    
    # Send sensor data to ESP32
    try:
        client_socket.sendall((sensor_message + '\n').encode('utf-8'))
        last_comm_time = current_time
    except socket.timeout:
        print("⚠️ Send timeout - ESP32 may be unresponsive")
        connection_established = False
        continue
    except Exception as e:
        print(f"❌ Send error: {e}")
        connection_established = False
        continue
    
    # Receive command from ESP32
    try:
        data = client_socket.recv(64)
        if not data:
            print("⚠️ ESP32 disconnected")
            connection_established = False
            continue
        
        command = data.decode('utf-8').strip()
        if command:
            # Validate command
            valid_commands = ['forward', 'turn_left_gentle', 'turn_left_sharp', 
                            'turn_right_gentle', 'turn_right_sharp', 'search', 'stop']
            if command in valid_commands:
                current_state = command
                last_comm_time = current_time
            else:
                print(f"⚠️ Invalid command: {command}")
        
    except socket.timeout:
        # No command received - continue with last state
        pass
    except Exception as e:
        print(f"❌ Receive error: {e}")
        connection_established = False
        continue
    
    # Safety timeout - stop if no communication for too long
    if current_time - last_comm_time > 3.0:
        print("⚠️ Communication timeout - stopping robot")
        current_state = 'stop'
    
    # Convert state to motor speeds
    left_speed = 0.0
    right_speed = 0.0
    
    if current_state == 'forward':
        left_speed = MAX_FORWARD_SPEED
        right_speed = MAX_FORWARD_SPEED
    elif current_state == 'turn_left_gentle':
        left_speed = GENTLE_TURN_INNER_SPEED
        right_speed = GENTLE_TURN_OUTER_SPEED
    elif current_state == 'turn_left_sharp':
        left_speed = -SHARP_TURN_ROTATE_SPEED
        right_speed = SHARP_TURN_ROTATE_SPEED
    elif current_state == 'turn_right_gentle':
        left_speed = GENTLE_TURN_OUTER_SPEED
        right_speed = GENTLE_TURN_INNER_SPEED
    elif current_state == 'turn_right_sharp':
        left_speed = SHARP_TURN_ROTATE_SPEED
        right_speed = -SHARP_TURN_ROTATE_SPEED
    elif current_state == 'search':
        left_speed = SEARCH_ROTATE_SPEED
        right_speed = -SEARCH_ROTATE_SPEED
    elif current_state == 'stop':
        left_speed = 0.0
        right_speed = 0.0
    
    # Apply motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# Cleanup
if client_socket:
    client_socket.close()
    print("Client socket closed.")

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
print("Robot controller finished.")