# main.py (ESP32 Code) - Fixed Dijkstra Path Planning with HIL

import network
import socket
import time
from machine import Pin
from path_planner import PathPlanner

# --- Wi-Fi Configuration ---
WIFI_SSID = "CJ"  # Change this
WIFI_PASSWORD = "4533simon"  # Change this

# --- Server Configuration ---
HOST_IP = '0.0.0.0' 
PORT = 8266

# --- Onboard LED ---
try:
    onboard_led = Pin(2, Pin.OUT)
    onboard_led.off() 
except Exception as e:
    print(f"Warning: Could not initialize onboard LED: {e}")
    onboard_led = None

def blink_led(pin, times=1, delay_on=0.1, delay_off=0.1):
    if pin:
        for _ in range(times):
            pin.on()
            time.sleep(delay_on)
            pin.off()
            time.sleep(delay_off)

# --- Wi-Fi Connection ---
def connect_wifi(ssid, password):
    station = network.WLAN(network.STA_IF)
    station.active(True)
    if not station.isconnected():
        print(f"Connecting to Wi-Fi (SSID: {ssid})...")
        station.connect(ssid, password)
        connect_attempts = 0
        while not station.isconnected() and connect_attempts < 20:
            print(".", end="")
            blink_led(onboard_led, 1, 0.1, 0.4)
            connect_attempts += 1
            time.sleep(0.5)
    
    if station.isconnected():
        ip_address = station.ifconfig()[0]
        print(f"\nWi-Fi Connected! ESP32 IP Address: {ip_address}")
        blink_led(onboard_led, 3, 0.1, 0.1)
        return station, ip_address
    else:
        print("\nFailed to connect to Wi-Fi.")
        blink_led(onboard_led, 5, 0.5, 0.1)
        return None, None

# --- Robot State Variables ---
line_left = False
line_center = False
line_right = False
current_state = 'stop'
loop_counter = 0

# --- Path Planning Setup ---
# Define your start and goal positions (grid coordinates)
START_NODE = (0, 0)      # Change these to your desired start/goal
GOAL_NODE = (12, 16)     # positions in the grid

path_planner = PathPlanner()
current_path = []
path_index = 0
at_intersection = False
last_intersection_time = 0
INTERSECTION_COOLDOWN = 2000  # ms to prevent multiple intersection detections

print(f"Planning path from {START_NODE} to {GOAL_NODE}")
planned_path = path_planner.find_path(START_NODE, GOAL_NODE)
if planned_path:
    print(f"Path found with {len(planned_path)} nodes: {planned_path}")
    current_path = planned_path
else:
    print("No path found! Check your start/goal positions.")

def get_next_direction():
    """Get the next movement direction based on current position in path"""
    global path_index, current_path
    
    if not current_path or path_index >= len(current_path) - 1:
        return 'stop'  # Reached end or no path
    
    current_pos = current_path[path_index]
    next_pos = current_path[path_index + 1]
    
    # Calculate direction vector
    dx = next_pos[1] - current_pos[1]  # Column difference (x-direction)
    dy = next_pos[0] - current_pos[0]  # Row difference (y-direction)
    
    # Map direction to robot command
    if dx == 1:      # Moving right
        return 'turn_right_gentle'
    elif dx == -1:   # Moving left
        return 'turn_left_gentle'
    elif dy == 1:    # Moving down
        return 'forward'
    elif dy == -1:   # Moving up
        return 'forward'
    else:
        return 'forward'  # Default case

def update_path_progress():
    """Update progress along the planned path when at intersections"""
    global path_index, at_intersection, last_intersection_time
    
    current_time = time.ticks_ms()
    
    # Detect intersection (all sensors see line)
    if line_left and line_center and line_right:
        if not at_intersection and time.ticks_diff(current_time, last_intersection_time) > INTERSECTION_COOLDOWN:
            at_intersection = True
            last_intersection_time = current_time
            
            # Advance to next node in path
            if path_index < len(current_path) - 1:
                path_index += 1
                print(f"Reached intersection! Moving to node {path_index}: {current_path[path_index]}")
            else:
                print("Reached final destination!")
    else:
        at_intersection = False

# --- Start Wi-Fi Connection ---
wlan_station, esp32_ip = connect_wifi(WIFI_SSID, WIFI_PASSWORD)

if wlan_station and wlan_station.isconnected():
    server_socket = None
    try:
        addr = socket.getaddrinfo(HOST_IP, PORT)[0][-1]
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(addr)
        server_socket.listen(1)
        print(f"TCP Server listening on {esp32_ip}:{PORT}")
        if onboard_led: onboard_led.on()
    except OSError as e:
        print(f"Error creating server socket: {e}")
        if server_socket: server_socket.close()
        server_socket = None

    while server_socket:
        client_socket = None
        client_address = None
        try:
            print("Waiting for Webots to connect...")
            client_socket, client_address = server_socket.accept()
            print(f"Webots connected from: {client_address}")
            if onboard_led: onboard_led.off()
            blink_led(onboard_led, 2, 0.05, 0.05)

            # Send initial state
            try:
                client_socket.sendall((current_state + '\n').encode('utf-8'))
            except OSError as e:
                print(f"Error sending initial command: {e}")
                raise

            while True:  # Main communication loop
                loop_counter += 1
                
                # 1. Receive sensor data from Webots
                try:
                    client_socket.settimeout(5.0)
                    data_bytes = client_socket.recv(128)
                    client_socket.settimeout(None)

                    if not data_bytes:
                        print("Webots disconnected.")
                        break 
                    
                    sensor_msg = data_bytes.decode('utf-8').strip()
                    
                    # Parse the message format: "LCR|OBS|x,y,theta"
                    parts = sensor_msg.split('|')
                    if len(parts) >= 1:
                        line_data = parts[0]
                        if len(line_data) == 3:
                            line_left = (line_data[0] == '1')
                            line_center = (line_data[1] == '1')
                            line_right = (line_data[2] == '1')

                except OSError as e:
                    print(f"Error receiving data: {e}")
                    break 
                except Exception as e:
                    print(f"Generic error receiving data: {e}")
                    break
                
                # 2. Update path progress
                update_path_progress()
                
                # 3. Determine robot state using line following + Dijkstra guidance
                previous_state = current_state
                
                if path_index >= len(current_path) - 1:
                    # Reached destination
                    current_state = 'stop'
                elif not line_left and line_center and not line_right:  # 010 - On line
                    current_state = 'forward'
                elif line_left and line_center and not line_right:   # 110 - Gentle left
                    next_dir = get_next_direction()
                    current_state = next_dir if next_dir == 'turn_left_gentle' else 'forward'
                elif not line_left and line_center and line_right:   # 011 - Gentle right  
                    next_dir = get_next_direction()
                    current_state = next_dir if next_dir == 'turn_right_gentle' else 'forward'
                elif line_left and not line_center and not line_right: # 100 - Sharp left
                    current_state = 'turn_left_sharp'
                elif not line_left and not line_center and line_right: # 001 - Sharp right
                    current_state = 'turn_right_sharp'
                elif line_left and line_center and line_right:       # 111 - Intersection
                    # Use Dijkstra's guidance at intersections
                    current_state = get_next_direction()
                elif not line_left and not line_center and not line_right: # 000 - Lost
                    current_state = 'search'
                elif line_left and not line_center and line_right: # 101 - Confused
                    current_state = 'search'
                
                # Log state changes
                if current_state != previous_state or loop_counter % 200 == 1:
                    sensor_str = f"{int(line_left)}{int(line_center)}{int(line_right)}"
                    print(f"Sensors: {sensor_str}, Path: {path_index}/{len(current_path)-1}, State: {current_state}")

                # 4. Send command to Webots
                try:
                    client_socket.sendall((current_state + '\n').encode('utf-8'))
                except OSError as e:
                    print(f"Error sending command: {e}")
                    break 
                
                time.sleep(0.02)  # Control loop rate

        except OSError as e:
            print(f"Socket error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            if client_socket:
                client_socket.close()
            print("Waiting for new connection...")
            if onboard_led: onboard_led.on()
else:
    print("Wi-Fi not connected. Cannot start server.")
    while True: 
        blink_led(onboard_led, 1, 0.2, 0.2)