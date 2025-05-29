# main.py (ESP32 Code) - Hybrid: Dijkstra Grid Path + Line Following

import network
import socket
import time
import uheapq # For Dijkstra's priority queue
from machine import Pin
import uasyncio as asyncio

# --- Wi-Fi Configuration ---
WIFI_SSID = "CJ"  # <<<<<<<<<<< CHANGE THIS
WIFI_PASSWORD = "4533simon"  # <<<<<<<<<<< CHANGE THIS
# --------------------------

# --- Server Configuration ---
SERVER_HOST_IP = '0.0.0.0'
SERVER_PORT = 8266
ONBOARD_LED_PIN = 2
# -------------------------

# --- Grid Map Definition (Used for high-level path planning) ---
GRID_ROWS = 13
GRID_COLS = 17
world_grid = [ # 0: Free, 1: Obstacle
    [1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],[1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0],[0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0],[0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1]
]
movement_costs = [ # Cost to enter a cell
    [1]*GRID_COLS for _ in range(GRID_ROWS) # Default cost 1
]
# Example: make some cells more costly
if 0 <= 3 < GRID_ROWS: 
    if 0 <= 0 < GRID_COLS: movement_costs[3][0] = 2
    if 0 <= 8 < GRID_COLS: movement_costs[3][8] = 2
    if 0 <= 16 < GRID_COLS: movement_costs[3][min(16, GRID_COLS-1)] = 2
if 0 <= 9 < GRID_ROWS:
    if 0 <= 0 < GRID_COLS: movement_costs[9][0] = 2
    if 0 <= 8 < GRID_COLS: movement_costs[9][8] = 2
    if 0 <= 16 < GRID_COLS: movement_costs[9][min(16, GRID_COLS-1)] = 2
# -----------------------------------------------------------------------

# --- Line Following Parameters ---
MAX_FORWARD_SPEED_LF = 2.5  # Max speed for line following
GENTLE_TURN_OUTER_LF = MAX_FORWARD_SPEED_LF
GENTLE_TURN_INNER_LF = MAX_FORWARD_SPEED_LF * 0.3 # Made gentle turns more pronounced
SHARP_TURN_SPEED_LF_OUTER = 1.8
SHARP_TURN_SPEED_LF_INNER = -1.0 # Pivot for sharp turns
SEARCH_ROTATE_SPEED_LF = 1.5 # For line search, if implemented

# --- Global State Variables ---
g_robot_current_cell_r, g_robot_current_cell_c = -1, -1
g_goal_cell_r, g_goal_cell_c = -1, -1 # Dijkstra Final Goal
g_current_dijkstra_path = None
g_dijkstra_path_step_index = 0
g_needs_new_dijkstra_plan = True
g_last_gs_data = {'l':0, 'm':0, 'r':0} # Store last known ground sensor state

# --- Onboard LED Setup & Helper (same as before) ---
try:
    onboard_led = Pin(ONBOARD_LED_PIN, Pin.OUT) if ONBOARD_LED_PIN is not None else None
    if onboard_led: onboard_led.off()
except Exception as e:
    print(f"Warning: Onboard LED init failed: {e}")
    onboard_led = None

def blink_led(pin, times=1, delay_on=0.1, delay_off=0.1):
    if pin:
        for _ in range(times):
            pin.on(); time.sleep(delay_on)
            pin.off(); time.sleep(delay_off)

# --- Dijkstra's Algorithm (same as before) ---
def dijkstra_grid(grid, costs, start_cell, goal_cell):
    rows, cols = len(grid), len(grid[0])
    if not (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols and grid[start_cell[0]][start_cell[1]] == 0):
        print(f"Dijkstra Error: Start cell {start_cell} invalid. GridVal: {grid[start_cell[0]][start_cell[1]] if (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols) else 'OOB'}")
        return None
    if not (0 <= goal_cell[0] < rows and 0 <= goal_cell[1] < cols and grid[goal_cell[0]][goal_cell[1]] == 0):
        print(f"Dijkstra Error: Goal cell {goal_cell} invalid. GridVal: {grid[goal_cell[0]][goal_cell[1]] if (0 <= goal_cell[0] < rows and 0 <= goal_cell[1] < cols) else 'OOB'}")
        return None
    pq = []; uheapq.heappush(pq, (0, start_cell))
    distances = {start_cell: 0}; predecessors = {start_cell: None}
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    while pq:
        dist, current_cell_tuple = uheapq.heappop(pq)
        if dist > distances.get(current_cell_tuple, float('inf')): continue
        if current_cell_tuple == goal_cell: break
        r_curr, c_curr = current_cell_tuple
        for dr, dc in directions:
            nr, nc = r_curr + dr, c_curr + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                move_cost = costs[nr][nc] if 0 <= nr < len(costs) and 0 <= nc < len(costs[0]) else 1
                new_distance = dist + move_cost
                if new_distance < distances.get((nr, nc), float('inf')):
                    distances[(nr, nc)] = new_distance
                    predecessors[(nr, nc)] = current_cell_tuple
                    uheapq.heappush(pq, (new_distance, (nr, nc)))
    if goal_cell not in distances or distances[goal_cell] == float('inf'):
        print(f"Dijkstra: No path from {start_cell} to {goal_cell}.")
        return None
    path = []; step = goal_cell
    while step is not None: path.append(step); step = predecessors.get(step)
    path.reverse()
    return path if path and path[0] == start_cell else None

# --- Wi-Fi Connection (same as before) ---
def connect_wifi(ssid, password):
    station = network.WLAN(network.STA_IF); station.active(True)
    if not station.isconnected():
        print(f"Connecting to Wi-Fi (SSID: {ssid})..."); station.connect(ssid, password)
        attempts = 0
        while not station.isconnected() and attempts < 20:
            print(".", end=""); blink_led(onboard_led, 1, 0.1, 0.4); attempts += 1; time.sleep(0.5)
    if station.isconnected():
        ip_address = station.ifconfig()[0]
        print(f"\nWi-Fi Connected! ESP32 IP Address: {ip_address}"); blink_led(onboard_led, 3, 0.1, 0.1)
        return station, ip_address
    else:
        print("\nFailed to connect to Wi-Fi."); blink_led(onboard_led, 5, 0.5, 0.1)
        return None, None

# --- TCP Server and Client Handling with Hybrid Logic ---
async def handle_client(reader, writer):
    global g_robot_current_cell_r, g_robot_current_cell_c, g_goal_cell_r, g_goal_cell_c
    global g_current_dijkstra_path, g_dijkstra_path_step_index, g_needs_new_dijkstra_plan, world_grid
    global g_last_gs_data

    client_addr = writer.get_extra_info('peername')
    print(f"Webots connected from: {client_addr}")
    if onboard_led: onboard_led.off(); blink_led(onboard_led, 2, 0.05, 0.05)

    g_robot_current_cell_r, g_robot_current_cell_c = -1, -1
    g_goal_cell_r, g_goal_cell_c = -1, -1
    g_current_dijkstra_path = None
    g_dijkstra_path_step_index = 0
    g_needs_new_dijkstra_plan = True
    g_last_gs_data = {'l':0, 'm':0, 'r':0} # Initialize to no line
    loop_iter_count = 0

    try:
        while True:
            loop_iter_count += 1
            command_to_send_webots = "stop" # Default command, will be overridden
            
            pos_data_processed_this_cycle = False
            gs_data_received_this_cycle = None 

            # 1. Receive and Parse Message from Webots
            try:
                raw_data = await asyncio.wait_for(reader.read(256), timeout=5.0)
                if not raw_data: print("Webots disconnected (empty data)."); break
                
                full_data_str = raw_data.decode('utf-8')
                lines = full_data_str.splitlines()
                
                for line_idx in range(len(lines) -1, -1, -1): 
                    current_line_cleaned = lines[line_idx].strip()
                    if not current_line_cleaned: continue

                    gs_part_str, pos_part_str, obstacle_part_str = None, None, None
                    
                    if current_line_cleaned.startswith("OBSTACLE:"):
                        obstacle_part_str = current_line_cleaned
                    elif "GS:" in current_line_cleaned and "POS:" in current_line_cleaned:
                        parts = current_line_cleaned.split(';', 1)
                        if parts[0].startswith("GS:"): gs_part_str = parts[0]
                        if len(parts) > 1 and parts[1].startswith("POS:"): pos_part_str = parts[1]
                    elif current_line_cleaned.startswith("GS:"): gs_part_str = current_line_cleaned
                    elif current_line_cleaned.startswith("POS:"): pos_part_str = current_line_cleaned

                    if obstacle_part_str:
                        try:
                            payload = obstacle_part_str.split(":",1)[1]; parts = payload.split(',')
                            if len(parts)==2:
                                obs_r, obs_c = int(parts[0]), int(parts[1])
                                if 0<=obs_r<GRID_ROWS and 0<=obs_c<GRID_COLS and world_grid[obs_r][obs_c]==0:
                                    world_grid[obs_r][obs_c]=1; print(f"ESP: Obstacle at ({obs_r},{obs_c}). Re-plan."); g_needs_new_dijkstra_plan=True
                        except Exception as eo: print(f"ESP Err parsing OBSTACLE: '{obstacle_part_str}', Ex: {eo}")
                    
                    if gs_part_str:
                        try:
                            gs_payload = gs_part_str.split(':',1)[1]; gs_values = [int(v) for v in gs_payload.split(',')]
                            if len(gs_values)==3: 
                                gs_data_received_this_cycle = {'l':gs_values[0],'m':gs_values[1],'r':gs_values[2]}
                                g_last_gs_data = gs_data_received_this_cycle 
                        except Exception as e_gs: print(f"ESP Err parsing GS: '{gs_part_str}', Ex: {e_gs}")
                    
                    if pos_part_str and not pos_data_processed_this_cycle:
                        try:
                            pos_payload = pos_part_str.split("POS:",1)[1]; parts = pos_payload.split(',')
                            if len(parts)==4:
                                temp_r,temp_c,temp_gr,temp_gc = int(parts[0]),int(parts[1]),int(parts[2]),int(parts[3])
                                if (g_robot_current_cell_r,g_robot_current_cell_c)!=(temp_r,temp_c) or \
                                   (g_goal_cell_r,g_goal_cell_c)!=(temp_gr,temp_gc) or \
                                   g_robot_current_cell_r==-1: g_needs_new_dijkstra_plan=True
                                g_robot_current_cell_r,g_robot_current_cell_c = temp_r,temp_c
                                g_goal_cell_r,g_goal_cell_c = temp_gr,temp_gc # This is the overall Dijkstra goal
                                pos_data_processed_this_cycle=True
                                if loop_iter_count % 10 == 0: print(f"ESP Parsed POS: Rob({temp_r},{temp_c}) Goal({temp_gr},{temp_gc}), NeedsPlan: {g_needs_new_dijkstra_plan}")
                            else: print(f"ESP Err: POS bad parts: {len(parts)} from '{pos_payload}'")
                        except Exception as ep: print(f"ESP Err parsing POS: '{pos_part_str}', Ex: {ep}")
                    
                    if pos_data_processed_this_cycle: break

            except asyncio.TimeoutError: pass
            except OSError as e: print(f"ESP Net Read OSError: {e}"); break 
            except Exception as e_recv: print(f"ESP Recv Error: {e_recv}"); break
            
            # 2. High-Level Dijkstra Path Planning
            if g_needs_new_dijkstra_plan and g_robot_current_cell_r != -1 and g_goal_cell_r != -1:
                print(f"ESP: Dijkstra Planning from ({g_robot_current_cell_r},{g_robot_current_cell_c}) to ({g_goal_cell_r},{g_goal_cell_c})")
                g_current_dijkstra_path = dijkstra_grid(world_grid, movement_costs, (g_robot_current_cell_r, g_robot_current_cell_c), (g_goal_cell_r, g_goal_cell_c))
                if g_current_dijkstra_path:
                    print(f"ESP: Dijkstra Path: {g_current_dijkstra_path}"); g_dijkstra_path_step_index = 0
                    path_str = "PATH_GRID:" + ";".join([f"{r},{c}" for r,c in g_current_dijkstra_path])
                    try: writer.write((path_str + '\n').encode('utf-8')); await writer.drain()
                    except Exception as e_send_path: print(f"Error sending path string: {e_send_path}"); break 
                else: print(f"ESP: No Dijkstra path found.")
                g_needs_new_dijkstra_plan = False

            # 3. Determine Motor Speeds (Line Following or Grid Navigation Fallback)
            s_l = g_last_gs_data['l']
            s_m = g_last_gs_data['m']
            s_r = g_last_gs_data['r']
            
            left_speed, right_speed = 0.0, 0.0
            use_line_following_speeds = False

            # Line Following Logic - This takes precedence if a line is detected
            if not (s_l == 0 and s_m == 0 and s_r == 0): # If any sensor is on the line
                use_line_following_speeds = True
                if s_l == 0 and s_m == 1 and s_r == 0: # 010: Forward
                    left_speed, right_speed = MAX_FORWARD_SPEED_LF, MAX_FORWARD_SPEED_LF
                elif s_l == 1 and s_m == 1 and s_r == 0: # 110: Gentle Left (robot too right)
                    left_speed, right_speed = GENTLE_TURN_INNER_LF, GENTLE_TURN_OUTER_LF
                elif s_l == 1 and s_m == 0 and s_r == 0: # 100: Sharp Left (robot too right)
                    left_speed, right_speed = SHARP_TURN_SPEED_LF_INNER, SHARP_TURN_SPEED_LF_OUTER
                elif s_l == 0 and s_m == 1 and s_r == 1: # 011: Gentle Right (robot too left)
                    left_speed, right_speed = GENTLE_TURN_OUTER_LF, GENTLE_TURN_INNER_LF
                elif s_l == 0 and s_m == 0 and s_r == 1: # 001: Sharp Right (robot too left)
                    left_speed, right_speed = SHARP_TURN_SPEED_LF_OUTER, SHARP_TURN_SPEED_LF_INNER
                elif s_l == 1 and s_m == 1 and s_r == 1: # 111: Intersection or thick line
                    left_speed, right_speed = MAX_FORWARD_SPEED_LF * 0.7, MAX_FORWARD_SPEED_LF * 0.7 # Slow down at intersection
                    # TODO: Future: Use Dijkstra path to decide turn at intersection
                else: # Other unhandled GS patterns (e.g., 101)
                    print(f"ESP: Unhandled GS pattern {s_l}{s_m}{s_r}. Defaulting to stop for line follow.")
                    left_speed, right_speed = 0.0, 0.0
            
            if use_line_following_speeds:
                command_to_send_webots = f"SPEEDS:{left_speed},{right_speed}"
            else: # No line detected (0,0,0) or unhandled pattern - Fallback to Dijkstra GOTO
                if g_current_dijkstra_path:
                    # Advance Dijkstra path step if robot is at the current target step of Dijkstra path
                    if g_dijkstra_path_step_index < len(g_current_dijkstra_path) and \
                       g_current_dijkstra_path[g_dijkstra_path_step_index] == (g_robot_current_cell_r, g_robot_current_cell_c):
                        print(f"ESP (Fallback GOTO): Reached Dijkstra step {g_dijkstra_path_step_index}: {g_current_dijkstra_path[g_dijkstra_path_step_index]}. Advancing.")
                        g_dijkstra_path_step_index += 1
                    
                    if g_dijkstra_path_step_index < len(g_current_dijkstra_path):
                        next_r_d, next_c_d = g_current_dijkstra_path[g_dijkstra_path_step_index]
                        command_to_send_webots = f"GOTO:{next_r_d},{next_c_d}"
                        print(f"ESP (Fallback GOTO): Line lost. Sending GOTO: ({next_r_d},{next_c_d})")
                    else: # Dijkstra path ended
                        if (g_robot_current_cell_r, g_robot_current_cell_c) == (g_goal_cell_r, g_goal_cell_c):
                            print("ESP (Fallback GOTO): Dijkstra Goal Reached! Sending STOP.")
                            command_to_send_webots = "stop" 
                            g_current_dijkstra_path = None 
                        else:
                            print("ESP (Fallback GOTO): Dijkstra Path ended, but not at overall goal. Sending STOP. Forcing re-plan.")
                            command_to_send_webots = "stop" 
                            g_current_dijkstra_path = None
                            g_needs_new_dijkstra_plan = True 
                else: # No line and no Dijkstra path
                    print("ESP (Fallback GOTO): No line and no Dijkstra path. Sending STOP.")
                    command_to_send_webots = "stop"
                    if g_robot_current_cell_r != -1 and g_goal_cell_r != -1 and \
                       (g_robot_current_cell_r, g_robot_current_cell_c) != (g_goal_cell_r, g_goal_cell_c):
                        if loop_iter_count % 5 == 0: # Try to replan more frequently if truly lost
                            g_needs_new_dijkstra_plan = True 

            # Check if overall goal reached (final override to stop)
            if (g_robot_current_cell_r, g_robot_current_cell_c) == (g_goal_cell_r, g_goal_cell_c) and g_goal_cell_r != -1 :
                print("ESP: Overall Final Goal Reached! Overriding to STOP.")
                command_to_send_webots = "stop" 
                g_current_dijkstra_path = None 

            # 4. Send Command to Webots
            try:
                if command_to_send_webots == "stop" or "GOTO" in command_to_send_webots or "SPEEDS:0.0,0.0" not in command_to_send_webots or loop_iter_count % 20 == 1 :
                    print(f"ESP TX CMD: '{command_to_send_webots}' (GS:{s_l}{s_m}{s_r}, L:{loop_iter_count}, Rob:({g_robot_current_cell_r},{g_robot_current_cell_c}), DijkstraGoal:({g_goal_cell_r},{g_goal_cell_c}), DijkstraNextStepIdx:{g_dijkstra_path_step_index})")
                
                writer.write((command_to_send_webots + '\n').encode('utf-8'))
                await writer.drain()
            except OSError as e_send_cmd: print(f"ESP Net Send OSError: {e_send_cmd}"); break 
            except Exception as e_send_cmd_other: print(f"ESP Send Error: {e_send_cmd_other}"); break

            await asyncio.sleep_ms(100)

    except Exception as e_main_loop: print(f"ESP32: **** Error in client loop: {e_main_loop} ****")
    finally:
        print(f"Closing connection with {client_addr}"); blink_led(onboard_led, 1, 0.5,0.1)
        try: writer.close(); await writer.wait_closed()
        except Exception as e_close: print(f"Error closing client socket: {e_close}")
        if onboard_led: onboard_led.on() 
        print("ESP32 waiting for new Webots connection...")

async def main_server():
    wlan, esp32_ip_addr = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    if not (wlan and wlan.isconnected()):
        print("ESP32: Wi-Fi not connected. Server cannot start.")
        while True: blink_led(onboard_led, 1, 0.2, 0.2); await asyncio.sleep(1) 
    print(f"TCP Server starting on {esp32_ip_addr}:{SERVER_PORT}")
    if onboard_led: onboard_led.on() 
    server = await asyncio.start_server(handle_client, SERVER_HOST_IP, SERVER_PORT)
    print(f"TCP Server listening on {esp32_ip_addr}:{SERVER_PORT}")
    while True: await asyncio.sleep(10)

if __name__ == "__main__":
    try: asyncio.run(main_server())
    except KeyboardInterrupt: print("ESP32: Program stopped by user.")
    except Exception as e: print(f"ESP32: Critical error in main: {e}")
    finally:
        if onboard_led: onboard_led.off()
        print("ESP32: Program terminated.")
