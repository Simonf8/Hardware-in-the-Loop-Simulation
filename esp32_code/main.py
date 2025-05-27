# main.py (ESP32 Code) - Wi-Fi HIL TCP Server - V2 (Continuous Command Sending)

import network
import socket
import time
from machine import Pin # For onboard LED status
from path_planner import PathPlanner  # Import our path planner

# --- Wi-Fi Configuration ---
WIFI_SSID = "CJ" # <<<<<<<<<<< CHANGE THIS
WIFI_PASSWORD = "4533simon" # <<<<<<<<<<< CHANGE THIS
# --------------------------

# --- Server Configuration ---
HOST_IP = '0.0.0.0' 
PORT = 8266         
# -------------------------

# --- Onboard LED (usually GPIO2 on many ESP32 boards) ---
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

# --- Wi-Fi Connection Function ---
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

# --- Main HIL Logic ---
line_left = False
line_center = False
line_right = False
current_state = 'stop' # Initial state
last_known_turn_direction = 'left'
loop_counter = 0

# Grid positions for path planning
current_grid_pos = (0, 0)  # Start position
goal_grid_pos = (12, 16)   # End position

# Initialize path planner
path_planner = PathPlanner()

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
        if onboard_led: onboard_led.on() # LED on solid while listening
    except OSError as e:
        print(f"Error creating server socket: {e}")
        if server_socket: server_socket.close()
        server_socket = None

    while server_socket:
        client_socket = None
        client_address = None
        try:
            print("Waiting for Webots (TCP client) to connect...")
            client_socket, client_address = server_socket.accept()
            print(f"Webots connected from: {client_address}")
            if onboard_led: onboard_led.off()
            blink_led(onboard_led, 2, 0.05, 0.05)

            # Send initial state once upon connection
            try:
                print(f"ESP Sending initial command to Webots: '{current_state}'")
                client_socket.sendall((current_state + '\n').encode('utf-8'))
            except OSError as e_init_send:
                print(f"Error sending initial command: {e_init_send}")
                raise # Re-raise to close client socket and re-listen

            while True: # Communication loop with this client
                loop_counter += 1
                # 1. Receive sensor data from Webots
                try:
                    client_socket.settimeout(5.0) # Timeout for receiving
                    data_bytes = client_socket.recv(128)
                    client_socket.settimeout(None) # Reset timeout

                    if not data_bytes:
                        print("Webots disconnected (received empty data).")
                        break 
                    
                    sensor_msg = data_bytes.decode('utf-8').strip()
                    if len(sensor_msg) == 3: # Expecting "LCR"
                        line_left = (sensor_msg[0] == '1')
                        line_center = (sensor_msg[1] == '1')
                        line_right = (sensor_msg[2] == '1')
                    else:
                        if sensor_msg: # Print if not empty but malformed
                             print(f"ESP Warning: Rcvd malformed sensor data: '{sensor_msg}'")
                        # If malformed, reuse previous sensor states or default to search?
                        # For now, just continue; previous sensor states will be used.
                        pass

                except OSError as e_recv:
                    print(f"Error receiving data or Webots timeout: {e_recv}")
                    break 
                except Exception as e_recv_gen:
                    print(f"Generic error receiving data: {e_recv_gen}")
                    break
                
                # 2. Parse sensor data from Webots
                sensor_data = sensor_msg.split('|')
                if len(sensor_data) >= 3:
                    # Parse line sensor data
                    line_sensor_data = sensor_data[0]
                    if len(line_sensor_data) == 3:
                        line_left = (line_sensor_data[0] == '1')
                        line_center = (line_sensor_data[1] == '1')
                        line_right = (line_sensor_data[2] == '1')
                    
                    # Parse obstacle data
                    obstacle_data = sensor_data[1]
                    path_planner.update_obstacles(obstacle_data)
                    
                    # Parse position data
                    try:
                        pos_data = sensor_data[2].split(',')
                        if len(pos_data) == 3:
                            x, y, theta = map(float, pos_data)
                            path_planner.update_position(x, y, theta)
                    except:
                        print("Error parsing position data")

                # 3. Determine Robot State using Path Planner
                previous_internal_state = current_state # For logging change
                
                if line_left and line_center and line_right:  # At intersection
                    # Update grid position based on previous moves
                    if current_state == 'forward':
                        if last_known_turn_direction == 'up':
                            current_grid_pos = (current_grid_pos[0]-1, current_grid_pos[1])
                        elif last_known_turn_direction == 'down':
                            current_grid_pos = (current_grid_pos[0]+1, current_grid_pos[1])
                        elif last_known_turn_direction == 'left':
                            current_grid_pos = (current_grid_pos[0], current_grid_pos[1]-1)
                        elif last_known_turn_direction == 'right':
                            current_grid_pos = (current_grid_pos[0], current_grid_pos[1]+1)
                
                # Get next move from path planner
                planned_move = path_planner.get_next_move(current_grid_pos, goal_grid_pos)
                
                # Combine path planning with line following behavior
                if not line_left and line_center and not line_right: # 010
                    current_state = 'forward'
                elif line_left and line_center and not line_right:   # 110
                    current_state = planned_move if planned_move == 'turn_left_gentle' else 'forward'
                    if current_state == 'turn_left_gentle':
                        last_known_turn_direction = 'left'
                elif not line_left and line_center and line_right:   # 011
                    current_state = planned_move if planned_move == 'turn_right_gentle' else 'forward'
                    if current_state == 'turn_right_gentle':
                        last_known_turn_direction = 'right'
                elif line_left and not line_center and not line_right: # 100
                    current_state = 'turn_left_sharp'
                    last_known_turn_direction = 'left'
                elif not line_left and not line_center and line_right: # 001
                    current_state = 'turn_right_sharp'
                    last_known_turn_direction = 'right'
                elif line_left and line_center and line_right:       # 111 (Junction)
                    # Use path planner's recommendation at intersections
                    current_state = planned_move
                    if planned_move == 'turn_left_gentle':
                        last_known_turn_direction = 'left'
                    elif planned_move == 'turn_right_gentle':
                        last_known_turn_direction = 'right'
                elif not line_left and not line_center and not line_right: # 000
                    current_state = 'search'
                elif line_left and not line_center and line_right: # 101
                    current_state = 'search'
                
                state_changed_this_cycle = (current_state != previous_internal_state)
                if state_changed_this_cycle or loop_counter % 200 == 1 :
                    print(f"ESP Logic: Sensors LCR={int(line_left)}{int(line_center)}{int(line_right)}, Prev='{previous_internal_state}', NewState='{current_state}'")

                # 3. ALWAYS Send the current determined state to Webots
                try:
                    # Optional: print only if state changed to reduce verbosity
                    # if state_changed_this_cycle:
                    #    print(f"ESP Sending command: '{current_state}'")
                    client_socket.sendall((current_state + '\n').encode('utf-8'))
                except OSError as e_send:
                    print(f"Error sending command (Webots likely disconnected): {e_send}")
                    break 
                except Exception as e_send_gen:
                    print(f"Generic error sending command: {e_send_gen}")
                    break
                
                time.sleep(0.02) # Loop rate for ESP32's decision making

        except OSError as e: # Errors from server_socket.accept() or initial send
            print(f"Socket accepting/initial send error: {e}")
        except Exception as e_outer:
            print(f"An unexpected error occurred in client handling: {e_outer}")
        finally:
            if client_socket:
                print("Closing client socket with Webots.")
                client_socket.close()
            print("ESP32 is now waiting for a new Webots connection...")
            if onboard_led: onboard_led.on() 
else:
    print("ESP32: Wi-Fi not connected. HIL Server cannot start.")
    while True: blink_led(onboard_led, 1, 0.2, 0.2)