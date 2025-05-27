# main.py (ESP32 Code) - Grid-Based Dijkstra Navigation via Wi-Fi - V12

import network
import socket
import time
import uheapq # For Dijkstra's priority queue
from machine import Pin

# --- Wi-Fi Configuration ---
WIFI_SSID = "YOUR_WIFI_SSID"        # <<<<<<<<<<< CHANGE THIS
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"  # <<<<<<<<<<< CHANGE THIS
# --------------------------

# --- Server Configuration ---
SERVER_HOST_IP = '0.0.0.0' 
SERVER_PORT = 8266
ONBOARD_LED_PIN = 2 # Common for ESP32 onboard LED, set to None if not using
# -------------------------

# --- Grid Map Definition (USER MUST DEFINE THIS ACCURATELY TO MATCH WEBOTS WORLD) ---
GRID_ROWS = 7  # Example: Number of rows in your grid
GRID_COLS = 9  # Example: Number of columns in your grid

# 0: Free space, 1: Obstacle
# This is a SMALL EXAMPLE GRID. You need to create one that accurately
# represents the free paths and obstacles of your Webots track.
world_grid = [
    [0, 0, 0, 1, 0, 0, 0, 0, 0], # Row 0
    [0, 1, 0, 1, 0, 1, 1, 1, 0], # Row 1
    [0, 1, 0, 0, 0, 0, 0, 1, 0], # Row 2
    [0, 1, 1, 1, 1, 1, 0, 1, 0], # Row 3
    [0, 0, 0, 0, 0, 1, 0, 1, 0], # Row 4
    [1, 1, 1, 1, 0, 1, 0, 0, 0], # Row 5
    [0, 0, 0, 0, 0, 0, 0, 0, 0]  # Row 6 <- Ensure your goal (e.g., 6,8) is 0
]

# Cost of moving into a cell (e.g., all 1s for uniform cost)
movement_costs = [[1 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
# -----------------------------------------------------------------------

# --- Global State Variables ---
g_robot_current_cell_r = -1 
g_robot_current_cell_c = -1
g_goal_cell_r = -1
g_goal_cell_c = -1
g_current_path = None
g_path_step_index = 0 # Which step of g_current_path the robot is aiming for next

# --- Onboard LED Setup & Helper ---
try:
    onboard_led = Pin(ONBOARD_LED_PIN, Pin.OUT) if ONBOARD_LED_PIN is not None else None
    if onboard_led: onboard_led.off()
except Exception as e: print(f"Warning: Onboard LED init failed: {e}"); onboard_led = None

def blink_led(pin, times=1, delay_on=0.1, delay_off=0.1):
    if pin:
        for _ in range(times): pin.on(); time.sleep(delay_on); pin.off(); time.sleep(delay_off)

# --- Dijkstra's Algorithm (Grid-based, using uheapq) ---
def dijkstra_grid(grid, costs, start_cell, goal_cell):
    rows, cols = len(grid), len(grid[0])
    # Validate start_cell
    if not (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols and grid[start_cell[0]][start_cell[1]] == 0):
        print(f"Dijkstra Error: Start cell {start_cell} is invalid or obstacle.")
        return None
    # Validate goal_cell
    if not (0 <= goal_cell[0] < rows and 0 <= goal_cell[1] < cols and grid[goal_cell[0]][goal_cell[1]] == 0):
        print(f"Dijkstra Error: Goal cell {goal_cell} is invalid or obstacle.")
        return None

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)] # Up, Down, Left, Right
    pq = []; uheapq.heappush(pq, (0, start_cell)) # (distance, (row, col))
    distances = {start_cell: 0}
    predecessors = {start_cell: None}

    while pq:
        current_dist, current_cell = uheapq.heappop(pq)
        if current_dist > distances.get(current_cell, float('inf')): continue
        if current_cell == goal_cell: break

        r, c = current_cell
        for dr, dc in directions:
            neighbor_r, neighbor_c = r + dr, c + dc
            if 0 <= neighbor_r < rows and 0 <= neighbor_c < cols and \
               grid[neighbor_r][neighbor_c] == 0:
                new_dist = current_dist + costs[neighbor_r][neighbor_c]
                if new_dist < distances.get((neighbor_r, neighbor_c), float('inf')):
                    distances[(neighbor_r, neighbor_c)] = new_dist
                    predecessors[(neighbor_r, neighbor_c)] = current_cell
                    uheapq.heappush(pq, (new_dist, (neighbor_r, neighbor_c)))
    
    if goal_cell not in distances or distances[goal_cell] == float('inf'):
        print(f"Dijkstra: No path found from {start_cell} to {goal_cell}.")
        return None

    path = []; step = goal_cell
    while step is not None: path.append(step); step = predecessors.get(step)
    path.reverse()
    return path if path[0] == start_cell else None

# --- Wi-Fi Connection ---
def connect_wifi(ssid, password):
    station = network.WLAN(network.STA_IF); station.active(True)
    if not station.isconnected():
        print(f"Connecting to Wi-Fi (SSID: {ssid})..."); station.connect(ssid, password)
        attempts=0
        while not station.isconnected() and attempts < 20: print(".",end=""); blink_led(onboard_led,1,0.1,0.4); attempts+=1; time.sleep(0.5)
    if station.isconnected():
        ip = station.ifconfig()[0]; print(f"\nWi-Fi Connected! ESP32 IP Address: {ip}"); blink_led(onboard_led,3,0.1,0.1); return station, ip
    print("\nFailed to connect to Wi-Fi."); blink_led(onboard_led,5,0.5,0.1); return None, None

# --- Main Program ---
wlan, esp32_ip_addr = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
if wlan and wlan.isconnected():
    server_sock = None
    try:
        addr_info=socket.getaddrinfo(SERVER_HOST_IP,SERVER_PORT)[0][-1]; server_sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); server_sock.bind(addr_info); server_sock.listen(1)
        print(f"TCP Server listening on {esp32_ip_addr}:{SERVER_PORT}");
        if onboard_led: onboard_led.on()
    except OSError as e: print(f"Error creating server socket: {e}"); server_sock = None

    while server_sock:
        client_sock = None
        g_current_path = None; g_path_step_index = 0
        g_robot_current_cell_r, g_robot_current_cell_c = -1, -1
        g_goal_cell_r, g_goal_cell_c = -1, -1
        needs_new_plan = True 
        
        try:
            print("Waiting for Webots (TCP client) to connect...")
            client_sock, client_addr = server_sock.accept()
            print(f"Webots connected from: {client_addr}")
            if onboard_led: onboard_led.off(); blink_led(onboard_led, 2, 0.05, 0.05)
            
            loop_iter_count = 0
            
            while True: # Main HIL communication loop
                try:
                    loop_iter_count +=1; command_to_send_webots = "stop" # Default command
                    
                    # 1. Receive Current Position and Goal from Webots
                    msg_from_webots_cleaned = ""
                    try:
                        client_sock.settimeout(0.2) # Shorter timeout, Webots should send POS frequently
                        all_data_bytes = b""
                        while True: # Read all buffered data for this cycle
                            try:
                                chunk = client_sock.recv(64)
                                if not chunk: all_data_bytes = None; break
                                all_data_bytes += chunk
                                if len(chunk) < 64: break 
                            except OSError as e_chunk_recv:
                                if e_chunk_recv.args[0] == 110: break # ETIMEDOUT on chunk
                                else: raise # Other socket error
                        client_sock.settimeout(None)

                        if all_data_bytes is None: print("Webots disconnected (recv empty chunk)."); break
                        
                        if all_data_bytes:
                            full_data_str = all_data_bytes.decode('utf-8')
                            lines = full_data_str.splitlines()
                            
                            for line in reversed(lines): # Process the LATEST POS message if multiple are stacked
                                msg_from_webots_cleaned = line.strip()
                                if not msg_from_webots_cleaned: continue

                                # print(f"ESP32 DEBUG: Processing line: '{msg_from_webots_cleaned}'") # Verbose
                                if msg_from_webots_cleaned.startswith("POS:"):
                                    try:
                                        payload = msg_from_webots_cleaned.split(":",1)[1]
                                        parts = payload.split(',')
                                        if len(parts) == 4:
                                            temp_robot_r = int(parts[0].strip())
                                            temp_robot_c = int(parts[1].strip())
                                            temp_goal_r = int(parts[2].strip())
                                            temp_goal_c = int(parts[3].strip())

                                            if (g_robot_current_cell_r, g_robot_current_cell_c) != (temp_robot_r, temp_robot_c) or \
                                               (g_goal_cell_r, g_goal_cell_c) != (temp_goal_r, temp_goal_c) or \
                                               g_robot_current_cell_r == -1: # Plan if current is uninit or anything changed
                                                needs_new_plan = True
                                            
                                            g_robot_current_cell_r, g_robot_current_cell_c = temp_robot_r, temp_robot_c
                                            g_goal_cell_r, g_goal_cell_c = temp_goal_r, temp_goal_c
                                            # print(f"ESP32 Parsed POS: Robot@({g_robot_current_cell_r},{g_robot_current_cell_c}), Goal@({g_goal_cell_r},{g_goal_cell_c})")
                                            break # Processed latest valid POS line
                                        else:
                                            print(f"ESP32 Error: POS msg bad parts: {parts} (from '{msg_from_webots_cleaned}')")
                                    except Exception as e_parse:
                                        print(f"ESP32 Error parsing POS payload from '{msg_from_webots_cleaned}', Ex: {e_parse}")
                                elif msg_from_webots_cleaned: # Some other message
                                    print(f"ESP32 Rcvd non-POS: '{msg_from_webots_cleaned}'")
                                    
                    except OSError as e_recv:
                        if e_recv.args[0] != 110: print(f"ESP32 Recv OSError: {e_recv}"); break 
                        # else ETIMEDOUT is fine if no new data from Webots this cycle
                    except Exception as e_recv_gen: print(f"ESP32 Generic Recv Error: {e_recv_gen}"); break
                    
                    # 2. Plan Path if needed
                    if needs_new_plan and g_robot_current_cell_r != -1 and g_goal_cell_r != -1:
                        print(f"ESP32: Planning path from ({g_robot_current_cell_r},{g_robot_current_cell_c}) to ({g_goal_cell_r},{g_goal_cell_c})")
                        g_current_path = dijkstra_grid(world_grid, movement_costs, 
                                                       (g_robot_current_cell_r, g_robot_current_cell_c), 
                                                       (g_goal_cell_r, g_goal_cell_c))
                        if g_current_path:
                            print(f"ESP32: Path found: {g_current_path}")
                            g_path_step_index = 0 # Start at the beginning (current cell)
                            path_str = "PATH_GRID:" + ";".join([f"{r},{c}" for r,c in g_current_path])
                            try: client_sock.sendall((path_str + '\n').encode('utf-8'))
                            except Exception as e: print(f"Error sending path string: {e}")
                        else:
                            print(f"ESP32: No path found. Robot at ({g_robot_current_cell_r},{g_robot_current_cell_c}). Goal ({g_goal_cell_r},{g_goal_cell_c})")
                        needs_new_plan = False
                    
                    # 3. Determine Next Move Command
                    if g_current_path:
                        # If robot is at the current step of the path, aim for the next step
                        if g_path_step_index < len(g_current_path) and \
                           g_current_path[g_path_step_index] == (g_robot_current_cell_r, g_robot_current_cell_c):
                            g_path_step_index += 1 # Advance to next target cell in path
                        
                        if g_path_step_index < len(g_current_path): # If there are more steps
                            next_r, next_c = g_current_path[g_path_step_index]
                            command_to_send_webots = f"GOTO:{next_r},{next_c}"
                            if loop_iter_count % 50 == 1: # Print periodically
                                print(f"ESP32: Current ({g_robot_current_cell_r},{g_robot_current_cell_c}), Next GOTO: ({next_r},{next_c})")
                        else: # Reached end of path steps
                            if (g_robot_current_cell_r, g_robot_current_cell_c) == (g_goal_cell_r, g_goal_cell_c):
                                print("ESP32: Goal Reached!")
                                command_to_send_webots = "stop" 
                                g_current_path = None # Clear path
                            else: # Path ended but not at physical goal cell
                                print("ESP32: Path ended, but not at goal cell. Stopping. Needs Replan?")
                                command_to_send_webots = "stop"; g_current_path = None; needs_new_plan = True
                    else: # No current path
                        command_to_send_webots = "stop"
                        if g_robot_current_cell_r != -1 and loop_iter_count % 100 == 1: # Valid current pos but no path
                            print(f"ESP32: No path to follow from ({g_robot_current_cell_r},{g_robot_current_cell_c}). Sending stop.")
                    
                    # 4. Send Command to Webots
                    try:
                        if loop_iter_count % 20 == 1 or command_to_send_webots == "stop": # Print stop or periodically
                             print(f"ESP32 Sending to Webots: '{command_to_send_webots}'")
                        client_sock.sendall((command_to_send_webots + '\n').encode('utf-8'))
                    except OSError as e_send: print(f"ESP32 Error sending command: {e_send}"); break
                    
                except Exception as e_main_loop_iter:
                    print(f"ESP32: **** Error in HIL iter: {e_main_loop_iter} ****")
                    try: client_sock.sendall(('stop' + '\n').encode('utf-8'))
                    except: pass
                    break 
                
                time.sleep(0.1) # ESP32 loop rate for grid navigation

        except OSError as e_accept: print(f"Socket error (accept/outer): {e_accept}")
        except Exception as e_outer_client_loop: print(f"Outer client handling error: {e_outer_client_loop}")
        finally:
            if client_sock: print("Closing client socket."); client_sock.close()
            print("ESP32 waiting for new Webots connection...")
            if onboard_led: onboard_led.on() 
else:
    print("ESP32: Wi-Fi not connected. HIL Server cannot start.")
    while True: blink_led(onboard_led, 1, 0.2, 0.2)