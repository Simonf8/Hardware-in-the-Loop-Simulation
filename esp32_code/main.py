# ESP32 MicroPython Controller with Dijkstra's Algorithm for Webots HIL
import network
import socket
import json
import time
import gc
from machine import Pin
import math

# --- WiFi Configuration ---
WIFI_SSID = ''        # Replace with your WiFi SSID
WIFI_PASSWORD = '' # Replace with your WiFi password
SERVER_PORT = 8080

# --- Onboard LED ---
led = Pin(2, Pin.OUT) # ESP32 onboard LED, usually GPIO2

# --- Grid Configuration (Must match Webots) ---
GRID_ROWS = 15
GRID_COLS = 19
# 0 = BLACK LINE (pathable)
# 1 = WHITE SPACE (obstacle)
grid_map = [
    [1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],  # Row 0
    [1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],  # Row 1
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 2
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0],  # Row 3
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0],  # Row 4
    [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0],  # Row 5
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0],  # Row 6
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 7
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0],  # Row 8
    [0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0],  # Row 9
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0],  # Row 10
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0],  # Row 11
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 12
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1],  # Row 13
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1]   # Row 14
]


# --- Path Planning State ---
current_robot_grid_pos_actual = None # Actual reported by Webots (row, col)
current_robot_grid_pos_path = None # Current position according to ESP32's path following (row, col)
goal_grid_pos = None
planned_path = [] # List of (row, col) tuples
current_path_index = 0
path_needs_replan = True
last_replan_time = 0
REPLAN_INTERVAL_MS = 1000 # Replan path if needed every 2 seconds, or if robot deviates

# --- Dijkstra's Algorithm Implementation ---
class SimplePriorityQueue:
    def __init__(self):
        self._queue = []

    def put(self, item, priority):
        self._queue.append({'item': item, 'priority': priority})
        self._queue.sort(key=lambda x: x['priority']) # Simple sort, can be optimized

    def get(self):
        if not self.is_empty():
            return self._queue.pop(0)['item']
        return None

    def is_empty(self):
        return len(self._queue) == 0

def get_valid_neighbors(r, c, rows, cols, grid):
    neighbors = []
    # (dr, dc) - cost is 1 for adjacent cells
    for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]: # Right, Left, Down, Up
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0: # Check bounds and if pathable
            neighbors.append(((nr, nc), 1)) # (neighbor_node, cost)
    return neighbors

def dijkstra(grid, start_node, end_node):
    rows, cols = len(grid), len(grid[0])
    # Validate start and end nodes
    if not (0 <= start_node[0] < rows and 0 <= start_node[1] < cols and grid[start_node[0]][start_node[1]] == 0):
        print(f"Dijkstra Error: Start node {start_node} is invalid or not on a pathable line.")
        return []
    if not (0 <= end_node[0] < rows and 0 <= end_node[1] < cols and grid[end_node[0]][end_node[1]] == 0):
        print(f"Dijkstra Error: End node {end_node} is invalid or not on a pathable line.")
        return []

    pq = SimplePriorityQueue()
    pq.put(start_node, 0) # (item, priority)
    
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
    
    path_found = False
    nodes_explored_count = 0

    while not pq.is_empty():
        current_node = pq.get()
        nodes_explored_count +=1

        if current_node == end_node:
            path_found = True
            break

        for next_node, cost in get_valid_neighbors(current_node[0], current_node[1], rows, cols, grid):
            new_cost = cost_so_far[current_node] + cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost # Dijkstra uses cost as priority
                pq.put(next_node, priority)
                came_from[next_node] = current_node
    
    if not path_found:
        print(f"Dijkstra: No path found from {start_node} to {end_node} after exploring {nodes_explored_count} nodes.")
        return []

    # Reconstruct path
    path = []
    node = end_node
    while node is not None: # Check if node is in came_from or is start_node
        path.append(node)
        node = came_from.get(node) # Safely get, will be None if node is start_node and not in came_from
    path.reverse()
    
    if path[0] != start_node : # Should not happen if path found
        print(f"Dijkstra WARNING: Path does not start at start_node! Starts at {path[0]}")
        return []

    print(f"Dijkstra: Path from {start_node} to {end_node} has {len(path)} steps (explored {nodes_explored_count}).")
    return path


# --- WiFi Connection ---
def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print(f'Attempting to connect to WiFi SSID: {ssid}')
        wlan.connect(ssid, password)
        timeout = 10 # seconds
        while not wlan.isconnected() and timeout > 0:
            print('.', end='')
            led.value(not led.value()) # Blink LED
            time.sleep(1)
            timeout -= 1
    if wlan.isconnected():
        led.on() # Solid LED for connected
        print(f'\nWiFi Connected! IP address: {wlan.ifconfig()[0]}')
        return wlan
    else:
        led.off()
        print('\nWiFi Connection Failed.')
        return None

# --- Server Setup ---
def start_server(port):
    # Get address info for binding
    # The tuple structure is (family, type, proto, canonname, sockaddr)
    addr = socket.getaddrinfo('0.0.0.0', port)[0][-1]
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow address reuse
    s.bind(addr)
    s.listen(1) # Listen for one connection
    print(f'ESP32 server listening on port {port}')
    return s

# --- Main Logic to Determine Action based on Path ---
def get_action_from_path(robot_pos_on_path, world_theta_rad, webots_line_sensors_binary):
    global planned_path, current_path_index, goal_grid_pos

    if not planned_path or not robot_pos_on_path: # No path or current position on path is unknown
        return 'stop', robot_pos_on_path

    if robot_pos_on_path == goal_grid_pos: # Goal reached
        return 'stop', robot_pos_on_path 

    # Ensure current_path_index is valid
    if not (0 <= current_path_index < len(planned_path) -1 ): # -1 because we need a next_node
        if current_path_index == len(planned_path) -1 and robot_pos_on_path == planned_path[-1]: # At the very last node (goal)
             return 'stop', robot_pos_on_path
        print(f"WARN: Path index {current_path_index} out of bounds for path length {len(planned_path)} or already at goal. Robot@Path: {robot_pos_on_path}")
        return 'stop', robot_pos_on_path # Recover by stopping, replan should fix


    current_node_on_path = planned_path[current_path_index]
    next_node_on_path = planned_path[current_path_index + 1]

    # Check if robot_pos_on_path matches the current_node_on_path expected by index
    if robot_pos_on_path != current_node_on_path:
        print(f"WARN: Robot's path position {robot_pos_on_path} differs from indexed path node {current_node_on_path}. Re-aligning index.")
        # Try to find robot_pos_on_path in the rest of the path
        try:
            current_path_index = planned_path.index(robot_pos_on_path, current_path_index) # Search from current index onwards
            current_node_on_path = planned_path[current_path_index]
            if current_path_index >= len(planned_path) -1: # Re-check after finding
                 return 'stop', robot_pos_on_path
            next_node_on_path = planned_path[current_path_index + 1]
        except ValueError:
            print(f"ERROR: Robot pos {robot_pos_on_path} not found in remaining path. Stopping. Replan needed.")
            return 'stop', robot_pos_on_path


    # Determine desired orientation based on path segment from current_node_on_path to next_node_on_path
    dr = next_node_on_path[0] - current_node_on_path[0]  # Change in row
    dc = next_node_on_path[1] - current_node_on_path[1]  # Change in col

    target_theta_rad = None
    if dc == 1 and dr == 0: target_theta_rad = 0.0           # Moving Right (+X)
    elif dc == -1 and dr == 0: target_theta_rad = math.pi    # Moving Left (-X)
    elif dr == 1 and dc == 0: target_theta_rad = math.pi / 2.0 # Moving Down (+Z, if rows increase downwards)
    elif dr == -1 and dc == 0: target_theta_rad = -math.pi / 2.0# Moving Up (-Z)
    else: 
        print(f"WARN: Non-adjacent nodes in path? {current_node_on_path} -> {next_node_on_path}")
        return 'stop', current_node_on_path

    # Normalize angles to -pi to pi
    current_theta_norm = math.atan2(math.sin(world_theta_rad), math.cos(world_theta_rad))
    # target_theta_rad is already between -pi/2 and pi.
    
    angle_diff = target_theta_rad - current_theta_norm
    # Normalize angle_diff to -pi to pi
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff)) 

    # print(f"Path: {current_node_on_path}->{next_node_on_path}. Curr Theta: {math.degrees(current_theta_norm):.1f}, Target Theta: {math.degrees(target_theta_rad):.1f}, Diff: {math.degrees(angle_diff):.1f}")

    ANGLE_THRESHOLD_RAD = math.radians(40) # Threshold to decide if we need to turn or go forward

    if abs(angle_diff) > ANGLE_THRESHOLD_RAD:
        # Positive diff means target is CCW from current -> turn left robot frame
        # Negative diff means target is CW from current -> turn right robot frame
        return 'turn_left' if angle_diff > 0 else 'turn_right', current_node_on_path
    else:
        # Angle is good, command forward
        return 'forward', current_node_on_path


# --- Main Program ---
if __name__ == "__main__":
    wlan = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    
    if not wlan or not wlan.isconnected():
        print("Stopping. No WiFi.")
    else:
        server_socket = start_server(SERVER_PORT)
        conn = None # Client connection

        while True:
            gc.collect() # Free up memory
            current_time_ms = time.ticks_ms()

            if conn is None:
                print("Waiting for Webots connection...")
                led.off() # Indicate waiting
                try:
                    server_socket.settimeout(1.0) # Timeout for accept
                    conn, addr = server_socket.accept()
                    conn.settimeout(0.1) # Non-blocking for recv
                    print(f"Connected by Webots: {addr}")
                    led.on() # Indicate connected
                    path_needs_replan = True # Force replan on new connection
                except OSError as e: # Timeout or other OS errors
                    if e.args[0] == 116: # ETIMEDOUT for MicroPython usocket (accept timeout)
                         pass # Normal timeout, continue waiting
                    else:
                         print(f"Accept error: {e}") # Other error during accept
                         conn = None # Ensure conn is None if accept failed for other reasons
                    time.sleep(0.5) # Wait a bit before retrying accept
                    continue # Retry accept
                except Exception as e: # Catch any other unexpected error during accept
                    print(f"Unexpected accept error: {e}")
                    conn = None
                    time.sleep(1) # Wait longer before retrying
                    continue


            # --- Receive Data from Webots ---
            try:
                data_bytes = conn.recv(512) # Adjust buffer size if paths are very long
                if data_bytes:
                    data_str = data_bytes.decode('utf-8').strip()
                    led.value(not led.value()) # Blink on receive to show activity

                    # Handle potentially multiple JSON objects if they are concatenated by TCP stream
                    for msg_part in data_str.split('\n'):
                        if not msg_part.strip(): continue # Skip empty parts
                        try:
                            webots_data = json.loads(msg_part)
                            # print(f"ESP RX: {webots_data}") # Verbose: log received data

                            if webots_data.get('type') == 'webots_status':
                                new_robot_pos_actual = tuple(webots_data.get('robot_grid_pos'))
                                new_goal_pos_from_webots = tuple(webots_data.get('goal_grid_pos'))
                                world_pose_data = webots_data.get('world_pose', {})
                                robot_theta_rad_from_webots = world_pose_data.get('theta_rad', 0.0)
                                line_sensors_binary_from_webots = webots_data.get('sensors_binary', [0,0,0])


                                if new_robot_pos_actual != current_robot_grid_pos_actual:
                                    # print(f"Robot moved from {current_robot_grid_pos_actual} to {new_robot_pos_actual}")
                                    current_robot_grid_pos_actual = new_robot_pos_actual
                                    # If robot has moved significantly from where our path *expects* it, consider replan
                                    if current_robot_grid_pos_path and \
                                       (abs(current_robot_grid_pos_actual[0] - current_robot_grid_pos_path[0]) > 1 or \
                                        abs(current_robot_grid_pos_actual[1] - current_robot_grid_pos_path[1]) > 1) : # Deviated by more than 1 cell
                                        print(f"Robot pos {current_robot_grid_pos_actual} deviates significantly from path pos {current_robot_grid_pos_path}. Forcing replan.")
                                        path_needs_replan = True
                                    
                                    # If current_robot_grid_pos_path is None, it's the first update or after replan
                                    if current_robot_grid_pos_path is None:
                                        current_robot_grid_pos_path = current_robot_grid_pos_actual


                                if new_goal_pos_from_webots != goal_grid_pos:
                                    goal_grid_pos = new_goal_pos_from_webots
                                    path_needs_replan = True
                                    print(f"New goal received from Webots: {goal_grid_pos}")
                                
                                # Initial setup for robot and goal if not set from a previous cycle
                                if current_robot_grid_pos_actual is None: current_robot_grid_pos_actual = new_robot_pos_actual
                                if current_robot_grid_pos_path is None: current_robot_grid_pos_path = new_robot_pos_actual
                                if goal_grid_pos is None: goal_grid_pos = new_goal_pos_from_webots; path_needs_replan = True # First time goal is set
                                

                                # --- Path Planning ---
                                if path_needs_replan or (time.ticks_diff(current_time_ms, last_replan_time) > REPLAN_INTERVAL_MS):
                                    if current_robot_grid_pos_actual and goal_grid_pos:
                                        print(f"Replanning path from {current_robot_grid_pos_actual} to {goal_grid_pos}...")
                                        gc.collect() # Collect garbage before intensive computation
                                        new_path = dijkstra(grid_map, current_robot_grid_pos_actual, goal_grid_pos)
                                        gc.collect() # And after
                                        
                                        if new_path:
                                            planned_path = new_path
                                            current_path_index = 0 # Start from the beginning of the new path
                                            
                                            # The first node in path should ideally be current_robot_grid_pos_actual.
                                            # If not, try to find where the robot is on this new path.
                                            if planned_path[0] == current_robot_grid_pos_actual:
                                                current_robot_grid_pos_path = planned_path[0]
                                            else:
                                                print(f"WARN: Dijkstra path starts at {planned_path[0]}, but robot is at {current_robot_grid_pos_actual}.")
                                                try: # See if the robot's current actual position is somewhere on the new path
                                                    current_path_index = planned_path.index(current_robot_grid_pos_actual)
                                                    current_robot_grid_pos_path = current_robot_grid_pos_actual
                                                    print(f"Robot found on new path at index {current_path_index}.")
                                                except ValueError: # robot_grid_pos_actual not in new_path
                                                    current_robot_grid_pos_path = planned_path[0] # Default to path start
                                                    current_path_index = 0
                                                    print("Robot's current actual pos not in new path. Starting path from its beginning.")
                                            
                                            path_needs_replan = False
                                            last_replan_time = current_time_ms
                                            # print(f"New path generated ({len(planned_path)} nodes). Robot on path @ {current_robot_grid_pos_path}, ESP Path Index: {current_path_index}.")
                                        else:
                                            print("Failed to generate new path. Will retry or stop if goal is invalid.")
                                            planned_path = [] # Clear old path
                                            path_needs_replan = True # Ensure retry, unless goal becomes unreachable
                                    else:
                                        print("Cannot replan: robot current position or goal position is unknown.")

                                # --- Determine Action to Send ---
                                action_to_send = 'stop' # Default action
                                if planned_path and current_robot_grid_pos_path and goal_grid_pos:
                                    action_to_send, _ = get_action_from_path( # Second return value (current_node_on_path) not directly used here
                                        current_robot_grid_pos_path, # Use ESP's understanding of robot's path position
                                        robot_theta_rad_from_webots,
                                        line_sensors_binary_from_webots
                                    )
                                    
                                    # --- Advance ESP's internal path state based on actual robot movement ---
                                    # If robot is at the node ESP thinks it should be at (current_robot_grid_pos_path)
                                    # AND the action is 'forward', it implies the robot should move to the next node.
                                    # We then check if the *actual* robot position (from Webots) has reached that *next* node.
                                    if current_robot_grid_pos_actual == current_robot_grid_pos_path:
                                        if action_to_send == 'forward' and current_path_index < len(planned_path) - 1:
                                            # We are telling it to go forward from current_robot_grid_pos_path.
                                            # The *next* node it should aim for is planned_path[current_path_index + 1].
                                            # If Webots reports robot_actual_pos IS this next_node, then advance our index.
                                            # This is a bit tricky because Webots takes time to move.
                                            # A better approach: ESP determines action. Webots moves.
                                            # On next cycle, ESP sees robot's new actual_pos. If actual_pos is the
                                            # next node in sequence, then ESP updates its current_robot_grid_pos_path and index.
                                            
                                            # Simplified advancement: If current_robot_grid_pos_actual matches current_robot_grid_pos_path,
                                            # and we are about to command 'forward', assume it will reach the next step.
                                            # Or, more robustly, update based on *past* successful movement.
                                            
                                            # Let's update current_robot_grid_pos_path and current_path_index
                                            # *after* confirming robot has reached the *next* target node.
                                            if current_path_index < len(planned_path) -1:
                                                prospective_next_node_on_path = planned_path[current_path_index+1]
                                                if current_robot_grid_pos_actual == prospective_next_node_on_path:
                                                    current_path_index +=1
                                                    current_robot_grid_pos_path = prospective_next_node_on_path
                                                    print(f"ESP: Robot confirmed at {current_robot_grid_pos_actual}. Advanced path index to {current_path_index} ({current_robot_grid_pos_path}).")


                                # Explicitly stop if actual position is the goal
                                if current_robot_grid_pos_actual == goal_grid_pos:
                                    action_to_send = 'stop'
                                    print("🎉 Goal Reached (actual pos matches goal)! Sending STOP.")
                                    planned_path = [] # Clear path as goal is reached, prevents re-planning to same spot.
                                    path_needs_replan = False # Don't immediately replan if at goal

                                # --- Send Command to Webots ---
                                command_to_webots = {
                                    'type': 'esp32_command',
                                    'action': action_to_send,
                                    'path': planned_path, # Send full path for Webots visualization
                                    'robot_pos_on_path_esp_thinks': list(current_robot_grid_pos_path) if current_robot_grid_pos_path else None,
                                    'current_path_idx_esp': current_path_index
                                }
                                response_json = json.dumps(command_to_webots) + '\n' # Ensure newline for Webots
                                conn.sendall(response_json.encode('utf-8'))
                                # print(f"ESP TX: Action: {action_to_send}, Path nodes: {len(planned_path)}, ESP Path Idx: {current_path_index}, ESP Path Pos: {current_robot_grid_pos_path}")

                        except json.JSONDecodeError as e:
                            print(f"JSON Decode Error from Webots: '{msg_part}', Error: {e}")
                        except Exception as e:
                            print(f"Error processing Webots message: {e} (Data: '{data_str[:100]}')") # Print first 100 chars of problematic data
                else: # Empty data usually means client disconnected gracefully
                    print("Webots disconnected (received empty data).")
                    conn.close()
                    conn = None
                    led.off()
                    # Reset state for next connection
                    current_robot_grid_pos_actual = None
                    current_robot_grid_pos_path = None
                    # goal_grid_pos = None # Keep goal unless Webots sends a new one or it's dynamic
                    planned_path = []
                    path_needs_replan = True


            except OSError as e: # socket.timeout or other OS errors during recv/send
                if e.args[0] == 116: # ETIMEDOUT for MicroPython on non-blocking recv (was 110)
                    pass # Normal, no data received within timeout
                elif e.args[0] == 104: # ECONNRESET (Connection reset by peer)
                    print("Webots connection reset by peer.")
                    if conn: conn.close()
                    conn = None
                    led.off()
                else: # Other OS errors
                    print(f"Socket recv/send OS error: {e}")
                    if conn: conn.close()
                    conn = None # Assume connection is broken
                    led.off()
            except Exception as e: # Other unexpected errors in the communication loop
                print(f"Main communication loop error: {e}")
                if conn: conn.close()
                conn = None # Assume connection is broken
                led.off()
                time.sleep(1) # Avoid rapid error loops if something is seriously wrong

            time.sleep(0.02) # Small delay in the main loop for stability and to yield