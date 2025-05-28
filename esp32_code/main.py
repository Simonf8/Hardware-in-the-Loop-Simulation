# main.py (ESP32 Code) - Grid-Based Dijkstra with Obstacle Updates

import network
import socket
import time
import uheapq # For Dijkstra's priority queue
from machine import Pin
import uasyncio as asyncio # For concurrent handling of server and potential obstacle messages

# --- Wi-Fi Configuration ---
WIFI_SSID = "YOUR_WIFI_SSID"  # <<<<<<<<<<< CHANGE THIS
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"  # <<<<<<<<<<< CHANGE THIS
# --------------------------

# --- Server Configuration ---
SERVER_HOST_IP = '0.0.0.0'
SERVER_PORT = 8266
ONBOARD_LED_PIN = 2  # Common for many ESP32 boards
# -------------------------

# --- Grid Map Definition ---
GRID_ROWS = 13
GRID_COLS = 17

# Initial world grid. This can be updated dynamically if obstacles are reported.
# 0: Free space, 1: Obstacle
world_grid = [
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 0 (bottom)
    [0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 1 - Cell (1,1) is 0
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 2
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 3
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 4
    [0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 5 - Cell (5,1) is NOW 0
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 6
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 7
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 8
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 9
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 10
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 11
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  # Row 12 (top) - Ensure goal (12,16) is 0
]
# Explicitly ensure the typical goal cell (12,16) is traversable
if 0 <= 12 < GRID_ROWS and 0 <= 16 < GRID_COLS:
    world_grid[12][16] = 0
# Explicitly ensure common start cells (0,0), (1,1) and now (5,1) are traversable
if 0 <= 0 < GRID_ROWS and 0 <= 0 < GRID_COLS:
    world_grid[0][0] = 0
if 0 <= 1 < GRID_ROWS and 0 <= 1 < GRID_COLS:
    world_grid[1][1] = 0
if 0 <= 5 < GRID_ROWS and 0 <= 1 < GRID_COLS:
    world_grid[5][1] = 0 # Corrected (5,1) to be navigable

# Movement costs (can also be dynamic if needed, but static for now)
movement_costs = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [2,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,2],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # Cost for row 5, col 1 is 1
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [2,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,2],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]
# You might want to ensure movement_costs[5][1] is also reasonable (e.g., 1) if it was different.
# The default above already has it as 1.
movement_costs[2][6] = 10
movement_costs[6][3] = 10
movement_costs[9][8] = 20
movement_costs[10][9] = 10
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
    """
    Finds the shortest path in a grid using Dijkstra's algorithm.
    Args:
        grid (list of lists): The grid map (0 for free, 1 for obstacle).
        costs (list of lists): The cost to move into each cell.
        start_cell (tuple): (row, col) of the starting cell.
        goal_cell (tuple): (row, col) of the goal cell.
    Returns:
        list of tuples: The path from start to goal, or None if no path.
    """
    rows, cols = len(grid), len(grid[0])

    # Validate start cell
    if not (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols and grid[start_cell[0]][start_cell[1]] == 0):
        print(f"Dijkstra Error: Start cell {start_cell} is invalid or an obstacle. Grid value: {grid[start_cell[0]][start_cell[1]]}")
        return None
    # Validate goal cell
    if not (0 <= goal_cell[0] < rows and 0 <= goal_cell[1] < cols and grid[goal_cell[0]][goal_cell[1]] == 0):
        print(f"Dijkstra Error: Goal cell {goal_cell} is invalid or an obstacle. Grid value: {grid[goal_cell[0]][goal_cell[1]]}")
        return None

    pq = []  # Priority queue: (distance, cell_tuple)
    uheapq.heappush(pq, (0, start_cell))

    distances = {start_cell: 0}  # Shortest distance from start to cell
    predecessors = {start_cell: None}  # Preceding cell in the shortest path

    # Directions: Up, Down, Left, Right (and optionally diagonals)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Row, Col changes

    while pq:
        dist, current_cell_tuple = uheapq.heappop(pq)

        if dist > distances.get(current_cell_tuple, float('inf')):
            continue # Already found a shorter path to this cell

        if current_cell_tuple == goal_cell:
            break  # Goal reached

        r_curr, c_curr = current_cell_tuple

        for dr, dc in directions:
            nr, nc = r_curr + dr, c_curr + dc  # Neighbor row, col

            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:  # Check bounds and if traversable
                # Cost to move to the neighbor cell
                move_cost = costs[nr][nc] # Cost to ENTER neighbor cell
                new_distance = dist + move_cost

                if new_distance < distances.get((nr, nc), float('inf')):
                    distances[(nr, nc)] = new_distance
                    predecessors[(nr, nc)] = current_cell_tuple
                    uheapq.heappush(pq, (new_distance, (nr, nc)))

    if goal_cell not in distances or distances[goal_cell] == float('inf'):
        print(f"Dijkstra: No path found from {start_cell} to {goal_cell}.")
        return None

    # Reconstruct path
    path = []
    step = goal_cell
    while step is not None:
        path.append(step)
        step = predecessors.get(step)
    path.reverse()

    return path if path and path[0] == start_cell else None


# --- Wi-Fi Connection ---
def connect_wifi(ssid, password):
    """Connects the ESP32 to Wi-Fi."""
    station = network.WLAN(network.STA_IF)
    station.active(True)
    if not station.isconnected():
        print(f"Connecting to Wi-Fi (SSID: {ssid})...")
        station.connect(ssid, password)
        attempts = 0
        while not station.isconnected() and attempts < 20: # 10 seconds timeout
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
        blink_led(onboard_led, 5, 0.5, 0.1) # Long blinks for failure
        return None, None

# --- TCP Server and Client Handling ---
async def handle_client(reader, writer):
    """Handles communication with a single Webots client."""
    global g_robot_current_cell_r, g_robot_current_cell_c, g_goal_cell_r, g_goal_cell_c
    global g_current_path, g_path_step_index, g_needs_new_plan, world_grid

    client_addr = writer.get_extra_info('peername')
    print(f"Webots connected from: {client_addr}")
    if onboard_led: onboard_led.off(); blink_led(onboard_led, 2, 0.05, 0.05)

    # Reset state for new client
    g_robot_current_cell_r, g_robot_current_cell_c = -1, -1
    g_goal_cell_r, g_goal_cell_c = -1, -1
    g_current_path = None
    g_path_step_index = 0
    g_needs_new_plan = True
    loop_iter_count = 0

    try:
        while True:
            loop_iter_count += 1
            command_to_send_webots = "stop" # Default command

            # 1. Receive Message from Webots (POS or OBSTACLE)
            try:
                # Set a timeout for reading to prevent indefinite blocking
                raw_data = await asyncio.wait_for(reader.read(256), timeout=5.0) # Increased buffer, 5s timeout
                if not raw_data:
                    print("Webots disconnected (empty data).")
                    break
                
                full_data_str = raw_data.decode('utf-8')
                lines = full_data_str.splitlines()
                pos_data_processed_this_cycle = False

                for line in reversed(lines): # Process latest relevant message first
                    current_line_cleaned = line.strip()
                    if not current_line_cleaned:
                        continue

                    if current_line_cleaned.startswith("POS:"):
                        if pos_data_processed_this_cycle: continue # Already got POS this cycle

                        # print(f"ESP RX POS: {current_line_cleaned}") # Can be noisy, enable for debug
                        pos_data_processed_this_cycle = True
                        try:
                            payload = current_line_cleaned.split(":", 1)[1]
                            parts = payload.split(',')
                            if len(parts) == 4:
                                temp_r, temp_c = int(parts[0]), int(parts[1])
                                temp_gr, temp_gc = int(parts[2]), int(parts[3])

                                # Check if position or goal changed, or if a new plan is explicitly needed
                                if (g_robot_current_cell_r, g_robot_current_cell_c) != (temp_r, temp_c) or \
                                   (g_goal_cell_r, g_goal_cell_c) != (temp_gr, temp_gc) or \
                                   g_robot_current_cell_r == -1 or g_needs_new_plan:
                                    g_needs_new_plan = True # Set flag to re-plan

                                g_robot_current_cell_r, g_robot_current_cell_c = temp_r, temp_c
                                g_goal_cell_r, g_goal_cell_c = temp_gr, temp_gc
                            else:
                                print(f"ESP Err: POS bad parts: {parts} (from '{current_line_cleaned}')")
                        except Exception as ep:
                            print(f"ESP Err parsing POS: '{current_line_cleaned}', Ex: {ep}")
                    
                    elif current_line_cleaned.startswith("OBSTACLE:"):
                        print(f"ESP RX OBSTACLE: {current_line_cleaned}")
                        try:
                            payload = current_line_cleaned.split(":", 1)[1]
                            parts = payload.split(',')
                            if len(parts) == 2:
                                obs_r, obs_c = int(parts[0]), int(parts[1])
                                if 0 <= obs_r < GRID_ROWS and 0 <= obs_c < GRID_COLS:
                                    if world_grid[obs_r][obs_c] == 0: # If it was previously free
                                        world_grid[obs_r][obs_c] = 1 # Mark as obstacle
                                        print(f"ESP: Updated grid. Obstacle at ({obs_r},{obs_c}). Re-planning needed.")
                                        g_needs_new_plan = True # Crucial: Force re-plan
                                    # else: # Already known or out of bounds, no need to print unless debugging
                                    #     print(f"ESP: Obstacle at ({obs_r},{obs_c}) already known or out of bounds.")
                                else:
                                    print(f"ESP Err: OBSTACLE coords out of bounds: ({obs_r},{obs_c})")
                            else:
                                print(f"ESP Err: OBSTACLE bad parts: {parts} (from '{current_line_cleaned}')")
                        except Exception as eo:
                            print(f"ESP Err parsing OBSTACLE: '{current_line_cleaned}', Ex: {eo}")
                    # Add other message types here if needed (e.g., "RESET_GRID:")

            except asyncio.TimeoutError:
                if loop_iter_count % 50 == 0:
                     pass
                pass
            except OSError as e: # Catch OSError for network issues like ConnectionResetError
                print(f"ESP Network OSError during read: {e}")
                break # Exit client handling loop
            except Exception as e_recv: # Catch any other unexpected errors during receive
                print(f"ESP Recv Error: {e_recv}")
                break
            
            # 2. Plan Path if needed
            if g_needs_new_plan and g_robot_current_cell_r != -1 and g_goal_cell_r != -1:
                print(f"ESP: Planning from ({g_robot_current_cell_r},{g_robot_current_cell_c}) to ({g_goal_cell_r},{g_goal_cell_c})")
                # Ensure the goal cell in the current world_grid is not an obstacle before planning
                if not (0 <= g_goal_cell_r < GRID_ROWS and 0 <= g_goal_cell_c < GRID_COLS and world_grid[g_goal_cell_r][g_goal_cell_c] == 0):
                    print(f"ESP: Goal ({g_goal_cell_r},{g_goal_cell_c}) is an obstacle or invalid in current grid. Cannot plan.")
                    g_current_path = None # Explicitly clear path
                else:
                    g_current_path = dijkstra_grid(world_grid, movement_costs,
                                               (g_robot_current_cell_r, g_robot_current_cell_c),
                                               (g_goal_cell_r, g_goal_cell_c))

                if g_current_path:
                    print(f"ESP: Path found: {g_current_path}")
                    g_path_step_index = 0 # Reset path index
                    path_str = "PATH_GRID:" + ";".join([f"{r},{c}" for r, c in g_current_path])
                    try:
                        writer.write((path_str + '\n').encode('utf-8'))
                        await writer.drain()
                        # print(f"ESP TX PATH: {path_str}") # Can be noisy
                    except OSError as e_send: # Catch OSError for network issues
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
                if g_path_step_index < len(g_current_path) and \
                   g_current_path[g_path_step_index] == (g_robot_current_cell_r, g_robot_current_cell_c):
                    g_path_step_index += 1
                
                if g_path_step_index < len(g_current_path):
                    next_r, next_c = g_current_path[g_path_step_index]
                    command_to_send_webots = f"GOTO:{next_r},{next_c}"
                else: 
                    if (g_robot_current_cell_r, g_robot_current_cell_c) == (g_goal_cell_r, g_goal_cell_c):
                        print("ESP: Goal Reached!")
                        command_to_send_webots = "stop"
                        g_current_path = None 
                    else:
                        print("ESP: Path ended, but not at the goal. Stopping. Re-planning might be needed.")
                        command_to_send_webots = "stop"
                        g_current_path = None
                        g_needs_new_plan = True 
            else: 
                command_to_send_webots = "stop"
                if g_robot_current_cell_r != -1 and g_goal_cell_r != -1 and \
                   (g_robot_current_cell_r, g_robot_current_cell_c) != (g_goal_cell_r, g_goal_cell_c):
                    if loop_iter_count % 10 == 0: 
                         g_needs_new_plan = True

            # 4. Send Command to Webots
            try:
                if command_to_send_webots == "stop" or loop_iter_count % 20 == 1 or "GOTO" in command_to_send_webots:
                    print(f"ESP TX CMD: '{command_to_send_webots}' (L:{loop_iter_count}, PIdx:{g_path_step_index}, NP:{g_needs_new_plan})")
                
                writer.write((command_to_send_webots + '\n').encode('utf-8'))
                await writer.drain()
            except OSError as e_send_cmd: # Catch OSError for network issues
                print(f"ESP Network OSError sending cmd: {e_send_cmd}")
                break 
            except Exception as e_send_cmd_other:
                print(f"ESP Error sending cmd: {e_send_cmd_other}")
                break

            await asyncio.sleep_ms(100)

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
        await asyncio.sleep(10)

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
