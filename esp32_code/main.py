# main.py (ESP32 Code) - Grid-Based Dijkstra with Obstacle Updates & Corrected Parser

import network
import socket
import time
import uheapq # For Dijkstra's priority queue
from machine import Pin
import uasyncio as asyncio # For concurrent handling of server and potential obstacle messages

# --- Wi-Fi Configuration ---
WIFI_SSID = "CJ"  # <<<<<<<<<<< CHANGE THIS
WIFI_PASSWORD = "4533simon"  # <<<<<<<<<<< CHANGE THIS
# --------------------------

# --- Server Configuration ---
SERVER_HOST_IP = '0.0.0.0'
SERVER_PORT = 8266
ONBOARD_LED_PIN = 2  # Common for many ESP32 boards
# -------------------------

# --- Grid Map Definition ---
GRID_ROWS = 13
GRID_COLS = 17

# Updated world grid to match your new layout (horizontally flipped to correct coordinate mapping)
# 0: Free space, 1: Obstacle
world_grid = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]   # Row 12 (top) - flipped
]

# Explicitly ensure common navigable cells are free
if 0 <= 0 < GRID_ROWS and 0 <= 0 < GRID_COLS:
    world_grid[0][0] = 0
if 0 <= 12 < GRID_ROWS and 0 <= 10 < GRID_COLS: # Assuming 10 is a valid target column
    world_grid[12][10] = 0

# Updated movement costs to match your new layout
movement_costs = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 0
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 1
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 2
    [2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2], # Row 3
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 4
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 5
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 6
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 7
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 8
    [2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2], # Row 9
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 10
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 11
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]  # Row 12
]

# Additional high-cost cells for specific locations (if desired)
if 0 <= 2 < GRID_ROWS and 0 <= 6 < GRID_COLS: movement_costs[2][6] = 10
if 0 <= 6 < GRID_ROWS and 0 <= 3 < GRID_COLS: movement_costs[6][3] = 10
if 0 <= 9 < GRID_ROWS and 0 <= 8 < GRID_COLS: movement_costs[9][8] = 20
if 0 <= 10 < GRID_ROWS and 0 <= 9 < GRID_COLS: movement_costs[10][9] = 10
# -----------------------------------------------------------------------

# --- Global State Variables ---
g_robot_current_cell_r, g_robot_current_cell_c = -1, -1
g_goal_cell_r, g_goal_cell_c = -1, -1
g_current_path = None
g_path_step_index = 0
g_needs_new_plan = True # Start by needing a plan

# --- Onboard LED Setup & Helper ---
try:
    onboard_led = Pin(ONBOARD_LED_PIN, Pin.OUT) if ONBOARD_LED_PIN is not None else None
    if onboard_led: onboard_led.off()
except Exception as e:
    print(f"Warning: Onboard LED init failed: {e}")
    onboard_led = None

def blink_led(pin, times=1, delay_on=0.1, delay_off=0.1):
    if pin:
        for _ in range(times):
            pin.on()
            time.sleep(delay_on)
            pin.off()
            time.sleep(delay_off)

# --- Dijkstra's Algorithm ---
def dijkstra_grid(grid, costs, start_cell, goal_cell):
    rows, cols = len(grid), len(grid[0])
    if not (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols and grid[start_cell[0]][start_cell[1]] == 0):
        print(f"Dijkstra Error: Start cell {start_cell} is invalid or an obstacle. Grid val: {grid[start_cell[0]][start_cell[1]] if (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols) else 'OOB'}")
        return None
    if not (0 <= goal_cell[0] < rows and 0 <= goal_cell[1] < cols and grid[goal_cell[0]][goal_cell[1]] == 0):
        print(f"Dijkstra Error: Goal cell {goal_cell} is invalid or an obstacle. Grid val: {grid[goal_cell[0]][goal_cell[1]] if (0 <= goal_cell[0] < rows and 0 <= goal_cell[1] < cols) else 'OOB'}")
        return None

    pq = []
    uheapq.heappush(pq, (0, start_cell))
    distances = {start_cell: 0}
    predecessors = {start_cell: None}
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while pq:
        dist, current_cell_tuple = uheapq.heappop(pq)
        if dist > distances.get(current_cell_tuple, float('inf')):
            continue
        if current_cell_tuple == goal_cell:
            break
        r_curr, c_curr = current_cell_tuple
        for dr, dc in directions:
            nr, nc = r_curr + dr, c_curr + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                move_cost = costs[nr][nc]
                new_distance = dist + move_cost
                if new_distance < distances.get((nr, nc), float('inf')):
                    distances[(nr, nc)] = new_distance
                    predecessors[(nr, nc)] = current_cell_tuple
                    uheapq.heappush(pq, (new_distance, (nr, nc)))
    if goal_cell not in distances or distances[goal_cell] == float('inf'):
        print(f"Dijkstra: No path found from {start_cell} to {goal_cell}.")
        return None
    path = []
    step = goal_cell
    while step is not None:
        path.append(step)
        step = predecessors.get(step)
    path.reverse()
    return path if path and path[0] == start_cell else None

# --- Wi-Fi Connection ---
def connect_wifi(ssid, password):
    station = network.WLAN(network.STA_IF)
    station.active(True)
    if not station.isconnected():
        print(f"Connecting to Wi-Fi (SSID: {ssid})...")
        station.connect(ssid, password)
        attempts = 0
        while not station.isconnected() and attempts < 20:
            print(".", end="")
            blink_led(onboard_led, 1, 0.1, 0.4)
            attempts += 1
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

# --- TCP Server and Client Handling (Corrected Parser) ---
async def handle_client(reader, writer):
    global g_robot_current_cell_r, g_robot_current_cell_c, g_goal_cell_r, g_goal_cell_c
    global g_current_path, g_path_step_index, g_needs_new_plan, world_grid

    client_addr = writer.get_extra_info('peername')
    print(f"Webots connected from: {client_addr}")
    if onboard_led: onboard_led.off(); blink_led(onboard_led, 2, 0.05, 0.05)

    g_robot_current_cell_r, g_robot_current_cell_c = -1, -1
    g_goal_cell_r, g_goal_cell_c = -1, -1
    g_current_path = None
    g_path_step_index = 0
    g_needs_new_plan = True
    loop_iter_count = 0

    try:
        while True:
            loop_iter_count += 1
            command_to_send_webots = "stop" 
            pos_data_processed_this_cycle = False
            gs_data_received_this_cycle = {} 

            try:
                raw_data = await asyncio.wait_for(reader.read(256), timeout=5.0)
                if not raw_data:
                    print("Webots disconnected (empty data).")
                    break
                
                full_data_str = raw_data.decode('utf-8')
                # Webots might send multiple messages if ESP is slow to read, or one combined message
                # Example: "OBSTACLE:r,c\nGS:l,m,r;POS:cr,cc,gr,gc\n" or "GS:l,m,r;POS:cr,cc,gr,gc\n"
                
                lines = full_data_str.splitlines()
                
                for line_idx in range(len(lines) -1, -1, -1): # Process from last complete line first
                    current_line_cleaned = lines[line_idx].strip()
                    if not current_line_cleaned:
                        continue

                    # Initialize parts for current line
                    gs_part_str = None
                    pos_part_str = None
                    obstacle_part_str = None

                    # Split the line by known message prefixes if they exist
                    # A single line could contain multiple message types if Webots concatenates them
                    # e.g. "OBSTACLE:r,c\nGS:l,m,r;POS:cr,cc,gr,gc"
                    # For simplicity, we assume Webots sends one primary message type per line OR
                    # a combined GS;POS message. If OBSTACLE is sent, it's usually on its own line.

                    if current_line_cleaned.startswith("OBSTACLE:"):
                        obstacle_part_str = current_line_cleaned
                    elif current_line_cleaned.startswith("GS:"):
                        parts = current_line_cleaned.split(';', 1)
                        gs_part_str = parts[0]
                        if len(parts) > 1 and "POS:" in parts[1]:
                            pos_part_str = parts[1] # POS part follows GS
                    elif "POS:" in current_line_cleaned: # Could be standalone POS:
                         # Check if it's part of a GS;POS structure that wasn't caught above
                        if ";" in current_line_cleaned and current_line_cleaned.split(';',1)[0].startswith("GS:"):
                            # Already handled by "GS:" check if combined
                            pass
                        else: # Likely standalone POS:
                            pos_part_str = current_line_cleaned


                    # Process Obstacle
                    if obstacle_part_str:
                        print(f"ESP RX OBSTACLE: {obstacle_part_str}")
                        try:
                            payload = obstacle_part_str.split(":", 1)[1]
                            parts = payload.split(',')
                            if len(parts) == 2:
                                obs_r, obs_c = int(parts[0]), int(parts[1])
                                if 0 <= obs_r < GRID_ROWS and 0 <= obs_c < GRID_COLS:
                                    if world_grid[obs_r][obs_c] == 0:
                                        world_grid[obs_r][obs_c] = 1
                                        print(f"ESP: Updated grid. Obstacle at ({obs_r},{obs_c}). Re-planning needed.")
                                        g_needs_new_plan = True
                                else:
                                    print(f"ESP Err: OBSTACLE coords out of bounds: ({obs_r},{obs_c})")
                            else:
                                print(f"ESP Err: OBSTACLE bad parts: {parts} (from '{obstacle_part_str}')")
                        except Exception as eo:
                            print(f"ESP Err parsing OBSTACLE: '{obstacle_part_str}', Ex: {eo}")
                    
                    # Process Ground Sensor Data
                    if gs_part_str:
                        try:
                            gs_payload = gs_part_str.split(':', 1)[1] 
                            gs_values = [int(v) for v in gs_payload.split(',')]
                            if len(gs_values) == 3:
                                gs_data_received_this_cycle = {'l': gs_values[0], 'm': gs_values[1], 'r': gs_values[2]}
                                if loop_iter_count % 10 == 0 : print(f"ESP Parsed GS: {gs_data_received_this_cycle}") # Debug less frequently
                            else:
                                print(f"ESP Err: GS bad parts count: {len(gs_values)} from '{gs_payload}'")
                        except Exception as e_gs:
                            print(f"ESP Err parsing GS part: '{gs_part_str}', Ex: {e_gs}")
                    
                    # Process Position Data (only once per data batch from Webots)
                    if pos_part_str and not pos_data_processed_this_cycle:
                        try:
                            pos_payload = pos_part_str.split("POS:", 1)[1] # Get content after "POS:"
                            parts = pos_payload.split(',')
                            if len(parts) == 4:
                                temp_r, temp_c = int(parts[0]), int(parts[1])
                                temp_gr, temp_gc = int(parts[2]), int(parts[3])

                                if (g_robot_current_cell_r, g_robot_current_cell_c) != (temp_r, temp_c) or \
                                   (g_goal_cell_r, g_goal_cell_c) != (temp_gr, temp_gc) or \
                                   g_robot_current_cell_r == -1: 
                                    g_needs_new_plan = True

                                g_robot_current_cell_r, g_robot_current_cell_c = temp_r, temp_c
                                g_goal_cell_r, g_goal_cell_c = temp_gr, temp_gc
                                pos_data_processed_this_cycle = True 
                                print(f"ESP Parsed POS: Rob({temp_r},{temp_c}) Goal({temp_gr},{temp_gc}), NeedsPlan: {g_needs_new_plan}")
                            else:
                                print(f"ESP Err: POS bad parts count: {len(parts)} from '{pos_payload}' (Line: '{current_line_cleaned}')")
                        except Exception as ep:
                            print(f"ESP Err parsing POS part: '{pos_part_str}', Ex: {ep}")
                    
                    if pos_data_processed_this_cycle: # If POS was found and processed, we're done with this line batch for POS
                        break


            except asyncio.TimeoutError:
                if loop_iter_count % 50 == 0:
                     print(f"ESP: Timeout waiting for Webots data. Last CMD: '{command_to_send_webots}'")
                pass
            except OSError as e: 
                print(f"ESP Network OSError during read: {e}")
                break 
            except Exception as e_recv: 
                print(f"ESP Recv Error: {e_recv}")
                break
            
            # 2. Plan Path if needed
            if g_needs_new_plan and g_robot_current_cell_r != -1 and g_goal_cell_r != -1:
                print(f"ESP: Planning from ({g_robot_current_cell_r},{g_robot_current_cell_c}) to ({g_goal_cell_r},{g_goal_cell_c})")
                if not (0 <= g_goal_cell_r < GRID_ROWS and 0 <= g_goal_cell_c < GRID_COLS and world_grid[g_goal_cell_r][g_goal_cell_c] == 0):
                    print(f"ESP: Goal ({g_goal_cell_r},{g_goal_cell_c}) is an obstacle or invalid in current grid. Cannot plan.")
                    g_current_path = None
                else:
                    g_current_path = dijkstra_grid(world_grid, movement_costs,
                                               (g_robot_current_cell_r, g_robot_current_cell_c),
                                               (g_goal_cell_r, g_goal_cell_c))
                if g_current_path:
                    print(f"ESP: Path found: {g_current_path}")
                    g_path_step_index = 0 
                    path_str = "PATH_GRID:" + ";".join([f"{r},{c}" for r, c in g_current_path])
                    try:
                        writer.write((path_str + '\n').encode('utf-8'))
                        await writer.drain()
                    except OSError as e_send:
                        print(f"ESP Network OSError sending path: {e_send}")
                        break
                    except Exception as e_send_path:
                        print(f"Error sending path string: {e_send_path}")
                        break 
                else:
                    print(f"ESP: No path found. Robot@({g_robot_current_cell_r},{g_robot_current_cell_c}), Goal@({g_goal_cell_r},{g_goal_cell_c})")
                g_needs_new_plan = False

            # 3. Determine Next Move Command
            if g_current_path:
                # If robot is at the current path step, advance to next step
                if g_path_step_index < len(g_current_path) and \
                   g_current_path[g_path_step_index] == (g_robot_current_cell_r, g_robot_current_cell_c):
                    print(f"ESP: Reached step {g_path_step_index}: {g_current_path[g_path_step_index]}. Advancing.")
                    g_path_step_index += 1
                
                if g_path_step_index < len(g_current_path):
                    next_r, next_c = g_current_path[g_path_step_index]
                    command_to_send_webots = f"GOTO:{next_r},{next_c}"
                else: # Path index is at or beyond the end of the path
                    if (g_robot_current_cell_r, g_robot_current_cell_c) == (g_goal_cell_r, g_goal_cell_c):
                        print("ESP: Goal Reached!")
                        command_to_send_webots = "stop"
                        g_current_path = None # Clear path as goal is reached
                    else:
                        # This case should ideally not happen if path planning and execution are correct
                        print("ESP: Path ended, but not at goal. Stopping. Forcing re-plan.")
                        command_to_send_webots = "stop"
                        g_current_path = None
                        g_needs_new_plan = True 
            else: # No current path
                command_to_send_webots = "stop"
                if g_robot_current_cell_r != -1 and g_goal_cell_r != -1 and \
                   (g_robot_current_cell_r, g_robot_current_cell_c) != (g_goal_cell_r, g_goal_cell_c):
                    if loop_iter_count % 10 == 0: # Periodically try to replan if stuck without a path
                         print("ESP: No current path, not at goal, attempting to trigger re-plan.")
                         g_needs_new_plan = True

            # 4. Send Command to Webots
            try:
                if command_to_send_webots == "stop" or "GOTO" in command_to_send_webots or loop_iter_count % 20 == 1 :
                    print(f"ESP TX CMD: '{command_to_send_webots}' (L:{loop_iter_count}, PIdx:{g_path_step_index}, NP:{g_needs_new_plan}, RobCur:({g_robot_current_cell_r},{g_robot_current_cell_c}), Goal:({g_goal_cell_r},{g_goal_cell_c}))")
                
                writer.write((command_to_send_webots + '\n').encode('utf-8'))
                await writer.drain()
            except OSError as e_send_cmd:
                print(f"ESP Network OSError sending cmd: {e_send_cmd}")
                break 
            except Exception as e_send_cmd_other:
                print(f"ESP Error sending cmd: {e_send_cmd_other}")
                break

            await asyncio.sleep_ms(100) # Main loop delay

    except Exception as e_main_loop:
        print(f"ESP32: **** Error in client handling loop: {e_main_loop} ****")
    finally:
        print(f"Closing connection with {client_addr}")
        try:
            writer.close()
            await writer.wait_closed()
        except Exception as e_close:
            print(f"Error closing client socket: {e_close}")
        if onboard_led: onboard_led.on() 
        print("ESP32 waiting for new Webots connection...")


async def main_server():
    """Main function to start the Wi-Fi, server, and handle clients."""
    wlan, esp32_ip_addr = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    if not (wlan and wlan.isconnected()):
        print("ESP32: Wi-Fi not connected. HIL Server cannot start.")
        while True: 
            blink_led(onboard_led, 1, 0.2, 0.2)
            await asyncio.sleep(1) 

    print(f"TCP Server starting on {esp32_ip_addr}:{SERVER_PORT}")
    if onboard_led: onboard_led.on() 

    server = await asyncio.start_server(handle_client, SERVER_HOST_IP, SERVER_PORT)
    
    print(f"TCP Server listening on {esp32_ip_addr}:{SERVER_PORT}")
    
    while True:
        await asyncio.sleep(10) # Keep the main_server task alive

if __name__ == "__main__":
    try:
        asyncio.run(main_server())
    except KeyboardInterrupt:
        print("ESP32: Program stopped by user.")
    except Exception as e:
        print(f"ESP32: Critical error in main: {e}")
    finally:
        if onboard_led: onboard_led.off()
        print("ESP32: Program terminated.")
