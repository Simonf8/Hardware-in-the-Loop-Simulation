# main.py (ESP32 Code) - Grid-Based Dijkstra with CORRECTED 13x17 Grid

import network
import socket
import time
import uheapq # For Dijkstra's priority queue
from machine import Pin

# --- Wi-Fi Configuration ---
WIFI_SSID = "CJ"        # <<<<<<<<<<< CHANGE THIS
WIFI_PASSWORD = "4533simon"  # <<<<<<<<<<< CHANGE THIS
# --------------------------

# --- Server Configuration ---
SERVER_HOST_IP = '0.0.0.0'
SERVER_PORT = 8266
ONBOARD_LED_PIN = 2
# -------------------------

# --- Grid Map Definition (User-Provided 13x17 Grid from Jupyter Notebook) ---
GRID_ROWS = 13
GRID_COLS = 17

world_grid = [ # 0: Free space, 1: Obstacle - Updated for bottom-left start layout
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 0 (bottom)
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 1
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 2
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 3
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 4
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 5
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 6
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 7
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 8
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 9
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 10
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 11
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]  # Row 12 (top) - The end point is on the right
]

movement_costs = [ # 
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [2,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,2],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [2,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,2], # Note: Notebook had 20 at col 8 (index) for some reason, using 2 from image.
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]

movement_costs[2][6] = 10
movement_costs[6][3] = 10
movement_costs[9][8] = 20 # This was the cell with cost 20 in notebook
movement_costs[10][9] = 10
# -----------------------------------------------------------------------

# --- Global State Variables ---
g_robot_current_cell_r, g_robot_current_cell_c = -1, -1
g_goal_cell_r, g_goal_cell_c = -1, -1
g_current_path = None 
g_path_step_index = 0

# --- Onboard LED Setup & Helper ---
try:
    onboard_led = Pin(ONBOARD_LED_PIN, Pin.OUT) if ONBOARD_LED_PIN is not None else None
    if onboard_led: onboard_led.off()
except Exception as e: print(f"Warning: Onboard LED init failed: {e}"); onboard_led = None

def blink_led(pin, times=1, delay_on=0.1, delay_off=0.1): # ... (same)
    if pin:
        for _ in range(times): pin.on(); time.sleep(delay_on); pin.off(); time.sleep(delay_off)

# --- Dijkstra's Algorithm (Identical to previous correct version) ---
def dijkstra_grid(grid, costs, start_cell, goal_cell): # ... (same)
    rows, cols = len(grid), len(grid[0])
    if not (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols and grid[start_cell[0]][start_cell[1]] == 0):
        print(f"Dijkstra Error: Start cell {start_cell} is invalid or obstacle."); return None
    if not (0 <= goal_cell[0] < rows and 0 <= goal_cell[1] < cols and grid[goal_cell[0]][goal_cell[1]] == 0):
        print(f"Dijkstra Error: Goal cell {goal_cell} is invalid or obstacle."); return None
    pq = []; uheapq.heappush(pq, (0, start_cell)); distances = {start_cell: 0}; predecessors = {start_cell: None}
    directions = [(-1,0),(1,0),(0,-1),(0,1)] # U, D, L, R
    while pq:
        dist, curr_cell_tuple = uheapq.heappop(pq) # Renamed curr_c to curr_cell_tuple
        if dist > distances.get(curr_cell_tuple, float('inf')): continue
        if curr_cell_tuple == goal_cell: break
        r_curr, c_curr = curr_cell_tuple # Unpack current cell for clarity
        for dr,dc in directions:
            nr,nc = r_curr+dr, c_curr+dc # Use r_curr, c_curr
            if 0<=nr<rows and 0<=nc<cols and grid[nr][nc]==0:
                new_d = dist + costs[nr][nc] # Cost to enter neighbor
                if new_d < distances.get((nr,nc), float('inf')):
                    distances[(nr,nc)]=new_d; predecessors[(nr,nc)]=curr_cell_tuple # Use curr_cell_tuple
                    uheapq.heappush(pq, (new_d, (nr,nc)))
    if goal_cell not in distances or distances[goal_cell]==float('inf'): print(f"Dijkstra: No path from {start_cell} to {goal_cell}."); return None
    path=[]; step=goal_cell
    while step is not None: path.append(step); step=predecessors.get(step)
    path.reverse(); return path if path and path[0]==start_cell else None

# --- Wi-Fi Connection (Identical to previous correct version) ---
def connect_wifi(ssid, password): # ... (same)
    station = network.WLAN(network.STA_IF); station.active(True)
    if not station.isconnected():
        print(f"Connecting to Wi-Fi (SSID: {ssid})..."); station.connect(ssid, password)
        attempts = 0
        while not station.isconnected() and attempts < 20: print(".", end=""); blink_led(onboard_led,1,0.1,0.4); attempts+=1; time.sleep(0.5)
    if station.isconnected():
        ip = station.ifconfig()[0]; print(f"\nWi-Fi Connected! ESP32 IP Address: {ip}"); blink_led(onboard_led,3,0.1,0.1); return station, ip
    print("\nFailed to connect to Wi-Fi."); blink_led(onboard_led,5,0.5,0.1); return None, None

# --- Main Program ---
wlan, esp32_ip_addr = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
if wlan and wlan.isconnected():
    server_sock = None
    try: # Server socket setup ... (Identical)
        addr_info=socket.getaddrinfo(SERVER_HOST_IP,SERVER_PORT)[0][-1]; server_sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); server_sock.bind(addr_info); server_sock.listen(1)
        print(f"TCP Server listening on {esp32_ip_addr}:{SERVER_PORT}");
        if onboard_led: onboard_led.on()
    except OSError as e: print(f"Error creating server socket: {e}"); server_sock = None

    while server_sock:
        client_sock = None; g_current_path = None; g_path_step_index = 0
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
                    loop_iter_count +=1; command_to_send_webots = "stop" 
                    
                    # 1. Receive POS message from Webots
                    msg_from_webots_cleaned = "" # Store the cleaned POS message
                    try: # ... (Receive logic using client_sock.recv and splitlines - same as V12) ...
                        client_sock.settimeout(0.2); all_data_bytes=b""
                        while True:
                            try:
                                chunk=client_sock.recv(64)
                                if not chunk:all_data_bytes=None;break
                                all_data_bytes+=chunk
                                if len(chunk)<64:break
                            except OSError as ech:
                                if ech.args[0]==110:break # ETIMEDOUT is fine
                                else:raise 
                        client_sock.settimeout(None)
                        if all_data_bytes is None:print("Webots disconnected (empty chunk).");break
                        if all_data_bytes:
                            full_data_str=all_data_bytes.decode('utf-8');lines=full_data_str.splitlines()
                            pos_data_proc=False
                            for line in reversed(lines): # Process latest POS first
                                current_line_cleaned=line.strip()
                                if not current_line_cleaned:continue
                                if current_line_cleaned.startswith("POS:"):
                                    pos_data_proc=True; msg_from_webots_cleaned = current_line_cleaned # Store for parsing
                                    try:
                                        payload=current_line_cleaned.split(":",1)[1];parts=payload.split(',')
                                        if len(parts)==4:
                                            temp_r,temp_c,temp_gr,temp_gc=int(parts[0]),int(parts[1]),int(parts[2]),int(parts[3])
                                            if(g_robot_current_cell_r,g_robot_current_cell_c)!=(temp_r,temp_c) or \
                                              (g_goal_cell_r,g_goal_cell_c)!=(temp_gr,temp_gc) or \
                                              g_robot_current_cell_r == -1 or needs_new_plan: # Plan if uninit, anything changed, or explicitly needed
                                                needs_new_plan=True
                                            g_robot_current_cell_r,g_robot_current_cell_c=temp_r,temp_c
                                            g_goal_cell_r,g_goal_cell_c=temp_gr,temp_gc
                                            break 
                                        else:print(f"ESP Err:POS bad parts:{parts}(from'{current_line_cleaned}')")
                                    except Exception as ep:print(f"ESP Err parse POS:'{current_line_cleaned}',Ex:{ep}")
                            # if not pos_data_proc and all_data_bytes and loop_iter_count%50==1:print(f"ESP Warn:Rcvd no POS:'{full_data_str[:50]}...'")
                    except OSError as er:
                        if er.args[0]!=110:print(f"ESP Recv OSError:{er}");break
                    except Exception as erg:print(f"ESP Generic Recv Err:{erg}");break
                    
                    # 2. Plan Path if needed
                    if needs_new_plan and g_robot_current_cell_r != -1 and g_goal_cell_r != -1:
                        print(f"ESP: Planning from ({g_robot_current_cell_r},{g_robot_current_cell_c}) to ({g_goal_cell_r},{g_goal_cell_c})")
                        g_current_path = dijkstra_grid(world_grid,movement_costs,(g_robot_current_cell_r,g_robot_current_cell_c),(g_goal_cell_r,g_goal_cell_c))
                        if g_current_path:
                            print(f"ESP: Path found: {g_current_path}"); g_path_step_index = 0
                            path_str="PATH_GRID:" + ";".join([f"{r},{c}" for r,c in g_current_path])
                            try:client_sock.sendall((path_str+'\n').encode('utf-8'))
                            except Exception as e:print(f"Error sending path string:{e}")
                        else:print(f"ESP: No path. Rob@({g_robot_current_cell_r},{g_robot_current_cell_c}).Goal({g_goal_cell_r},{g_goal_cell_c})")
                        needs_new_plan=False # Reset flag
                    
                    # 3. Determine Next Move Command
                    # ... (Movement logic identical to V12, sets command_to_send_webots) ...
                    if g_current_path:
                        if g_path_step_index < len(g_current_path) and \
                           g_current_path[g_path_step_index] == (g_robot_current_cell_r,g_robot_current_cell_c):
                            g_path_step_index += 1 
                        if g_path_step_index < len(g_current_path):
                            next_r,next_c = g_current_path[g_path_step_index]; command_to_send_webots = f"GOTO:{next_r},{next_c}"
                        else: 
                            if (g_robot_current_cell_r,g_robot_current_cell_c)==(g_goal_cell_r,g_goal_cell_c):print("ESP: Goal Reached!");command_to_send_webots="stop";g_current_path=None
                            else:print("ESP: Path end, not goal. Stop.");command_to_send_webots="stop";g_current_path=None;needs_new_plan=True
                    else: command_to_send_webots="stop"
                    
                    # 4. Send Command to Webots
                    try:
                        # Print command being sent for clarity, especially if it's 'stop' or changed
                        if command_to_send_webots == "stop" or loop_iter_count % 20 == 1 : 
                             print(f"ESP Sending: '{command_to_send_webots}' (Loop: {loop_iter_count})")
                        client_sock.sendall((command_to_send_webots + '\n').encode('utf-8'))
                    except OSError as e_send: print(f"ESP Error sending cmd: {e_send}"); break
                    
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