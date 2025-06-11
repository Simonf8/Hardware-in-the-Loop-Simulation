# ESP32 MicroPython Controller with A* Algorithm for Webots HIL
# Improved code following assignment rubric requirements

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
current_robot_grid_pos_path = None # Current position according to ESP32's path following
goal_grid_pos = None
planned_path = [] # List of (row, col) tuples
current_path_index = 0
path_needs_replan = True
last_replan_time = 0
REPLAN_INTERVAL_MS = 20000 # Replan path if needed every 2 seconds

# --- A* Algorithm Implementation ---
class AStarPriorityQueue:
    """Priority queue for A* algorithm with f-score priority"""
    def __init__(self):
        self._queue = []

    def put(self, item, f_score):
        """Add item with f_score priority (lower is better)"""
        self._queue.append({'item': item, 'f_score': f_score})
        # Keep queue sorted by f_score
        self._queue.sort(key=lambda x: x['f_score'])

    def get(self):
        """Get item with lowest f_score"""
        if not self.is_empty():
            return self._queue.pop(0)['item']
        return None

    def is_empty(self):
        """Check if queue is empty"""
        return len(self._queue) == 0

def manhattan_distance(node1, node2):
    """
    Calculate Manhattan distance heuristic between two nodes.
    This is admissible for grid-based pathfinding with 4-directional movement.
    
    Args:
        node1: (row, col) tuple
        node2: (row, col) tuple
    
    Returns:
        Manhattan distance between nodes
    """
    return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])

def euclidean_distance(node1, node2):
    """
    Calculate Euclidean distance heuristic between two nodes.
    This is also admissible but less tight than Manhattan for 4-directional movement.
    
    Args:
        node1: (row, col) tuple
        node2: (row, col) tuple
    
    Returns:
        Euclidean distance between nodes
    """
    dr = node1[0] - node2[0]
    dc = node1[1] - node2[1]
    return math.sqrt(dr * dr + dc * dc)

def get_valid_neighbors(r, c, rows, cols, grid):
    """
    Get valid neighboring cells that can be traversed.
    
    Args:
        r, c: Current position
        rows, cols: Grid dimensions
        grid: Grid map
    
    Returns:
        List of ((row, col), cost) tuples for valid neighbors
    """
    neighbors = []
    # 4-directional movement: Right, Left, Down, Up
    for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
            neighbors.append(((nr, nc), 1)) # Cost is 1 for adjacent cells
    return neighbors

def a_star(grid, start_node, end_node, heuristic='manhattan'):
    """
    A* pathfinding algorithm implementation.
    
    Args:
        grid: 2D grid map (0 = pathable, 1 = obstacle)
        start_node: (row, col) starting position
        end_node: (row, col) goal position
        heuristic: 'manhattan' or 'euclidean' heuristic function
    
    Returns:
        List of (row, col) tuples representing the shortest path, or empty list if no path exists
    """
    rows, cols = len(grid), len(grid[0])
    
    # Validate start and end nodes
    if not (0 <= start_node[0] < rows and 0 <= start_node[1] < cols and grid[start_node[0]][start_node[1]] == 0):
        print(f"A* Error: Start node {start_node} is invalid or not pathable.")
        return []
    if not (0 <= end_node[0] < rows and 0 <= end_node[1] < cols and grid[end_node[0]][end_node[1]] == 0):
        print(f"A* Error: End node {end_node} is invalid or not pathable.")
        return []

    # Select heuristic function
    h_func = manhattan_distance if heuristic == 'manhattan' else euclidean_distance
    
    # Initialize A* data structures
    open_set = AStarPriorityQueue()
    open_set.put(start_node, 0)
    
    came_from = {start_node: None}
    g_score = {start_node: 0}  # Cost from start to node
    f_score = {start_node: h_func(start_node, end_node)}  # g + h
    
    path_found = False
    nodes_explored = 0

    while not open_set.is_empty():
        current_node = open_set.get()
        nodes_explored += 1

        # Check if we reached the goal
        if current_node == end_node:
            path_found = True
            break

        # Explore neighbors
        for next_node, move_cost in get_valid_neighbors(current_node[0], current_node[1], rows, cols, grid):
            tentative_g_score = g_score[current_node] + move_cost
            
            # If we found a better path to next_node
            if next_node not in g_score or tentative_g_score < g_score[next_node]:
                # Update path information
                came_from[next_node] = current_node
                g_score[next_node] = tentative_g_score
                h_score = h_func(next_node, end_node)
                f_score[next_node] = tentative_g_score + h_score
                
                # Add to open set with f_score priority
                open_set.put(next_node, f_score[next_node])
    
    if not path_found:
        print(f"A*: No path found from {start_node} to {end_node} after exploring {nodes_explored} nodes.")
        return []

    # Reconstruct path
    path = []
    node = end_node
    while node is not None:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    
    print(f"A*: Path from {start_node} to {end_node} has {len(path)} steps (explored {nodes_explored} nodes).")
    return path

def detect_obstacles_from_distance_sensor(distance_value, robot_pos, robot_theta):
    """Simple obstacle detection for A* algorithm"""
    obstacles = []
    OBSTACLE_THRESHOLD = 0.1  # Obstacle detected if distance < 0.1m
    
    if distance_value < OBSTACLE_THRESHOLD and distance_value > 0.01:
        print(f"A* Algorithm: Distance sensor detected obstacle at {distance_value:.3f}m")
        # Simple obstacle detection - mark cell ahead as obstacle
        obstacle_distance_cells = 1
        
        dr, dc = 0, 0
        if -0.785 <= robot_theta <= 0.785:  # Facing right
            dc = obstacle_distance_cells
        elif 0.785 < robot_theta <= 2.356:  # Facing down
            dr = obstacle_distance_cells
        elif -2.356 <= robot_theta < -0.785:  # Facing up
            dr = -obstacle_distance_cells
        else:  # Facing left
            dc = -obstacle_distance_cells
        
        obstacle_row = robot_pos[0] + dr
        obstacle_col = robot_pos[1] + dc
        
        if 0 <= obstacle_row < GRID_ROWS and 0 <= obstacle_col < GRID_COLS:
            obstacles.append((obstacle_row, obstacle_col))
            # Mark as obstacle in grid
            if grid_map[obstacle_row][obstacle_col] == 0:
                grid_map[obstacle_row][obstacle_col] = 1
                print(f"A*: Marked ({obstacle_row}, {obstacle_col}) as obstacle")
    
    return obstacles

# --- WiFi Connection ---
def connect_wifi(ssid, password):
    """
    Connect to WiFi network.
    
    Args:
        ssid: WiFi network name
        password: WiFi password
    
    Returns:
        WLAN object if connected, None otherwise
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print(f'Attempting to connect to WiFi SSID: {ssid}')
        wlan.connect(ssid, password)
        timeout = 10  # seconds
        
        while not wlan.isconnected() and timeout > 0:
            print('.', end='')
            led.value(not led.value())  # Blink LED
            time.sleep(1)
            timeout -= 1
    
    if wlan.isconnected():
        led.on()  # Solid LED for connected
        print(f'\nWiFi Connected! IP address: {wlan.ifconfig()[0]}')
        return wlan
    else:
        led.off()
        print('\nWiFi Connection Failed.')
        return None

# --- Server Setup ---
def start_server(port):
    """
    Start TCP server for Webots communication.
    
    Args:
        port: Port number to listen on
    
    Returns:
        Socket object
    """
    addr = socket.getaddrinfo('0.0.0.0', port)[0][-1]
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    print(f'ESP32 server listening on port {port}')
    return s

# --- Main Logic to Determine Action based on Path ---
def get_action_from_path(robot_pos_on_path, world_theta_rad, webots_line_sensors_binary):
    """
    Determine robot action based on current position and planned path.
    
    Args:
        robot_pos_on_path: Current robot position on path
        world_theta_rad: Robot's current orientation in radians  
        webots_line_sensors_binary: Line sensor readings (unused in current implementation)
    
    Returns:
        Tuple of (action_string, current_position)
    """
    global planned_path, current_path_index, goal_grid_pos

    if not planned_path or not robot_pos_on_path:
        return 'stop', robot_pos_on_path

    if robot_pos_on_path == goal_grid_pos:
        return 'stop', robot_pos_on_path 

    # Ensure current_path_index is valid
    if not (0 <= current_path_index < len(planned_path) - 1):
        if current_path_index == len(planned_path) - 1 and robot_pos_on_path == planned_path[-1]:
            return 'stop', robot_pos_on_path
        print(f"WARN: Path index {current_path_index} out of bounds. Robot@Path: {robot_pos_on_path}")
        return 'stop', robot_pos_on_path

    current_node_on_path = planned_path[current_path_index]
    next_node_on_path = planned_path[current_path_index + 1]

    # Re-align path index if robot position doesn't match expected
    if robot_pos_on_path != current_node_on_path:
        print(f"WARN: Robot pos {robot_pos_on_path} differs from expected {current_node_on_path}. Re-aligning.")
        try:
            current_path_index = planned_path.index(robot_pos_on_path, current_path_index)
            current_node_on_path = planned_path[current_path_index]
            if current_path_index >= len(planned_path) - 1:
                return 'stop', robot_pos_on_path
            next_node_on_path = planned_path[current_path_index + 1]
        except ValueError:
            print(f"ERROR: Robot pos {robot_pos_on_path} not found in path. Stopping.")
            return 'stop', robot_pos_on_path

    # Determine target orientation based on next move
    dr = next_node_on_path[0] - current_node_on_path[0]
    dc = next_node_on_path[1] - current_node_on_path[1]

    target_theta_rad = None
    if dc == 1 and dr == 0: 
        target_theta_rad = 0.0  # Moving Right (+X)
    elif dc == -1 and dr == 0: 
        target_theta_rad = math.pi  # Moving Left (-X)
    elif dr == 1 and dc == 0: 
        target_theta_rad = math.pi / 2.0  # Moving Down (+Z)
    elif dr == -1 and dc == 0: 
        target_theta_rad = -math.pi / 2.0  # Moving Up (-Z)
    else:
        print(f"WARN: Non-adjacent nodes in path? {current_node_on_path} -> {next_node_on_path}")
        return 'stop', current_node_on_path

    # Normalize angles to -pi to pi
    current_theta_norm = math.atan2(math.sin(world_theta_rad), math.cos(world_theta_rad))
    
    angle_diff = target_theta_rad - current_theta_norm
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    ANGLE_THRESHOLD_RAD = math.radians(40)

    if abs(angle_diff) > ANGLE_THRESHOLD_RAD:
        # Turn towards target orientation
        return 'turn_left' if angle_diff > 0 else 'turn_right', current_node_on_path
    else:
        # Aligned with target, move forward
        return 'forward', current_node_on_path

# --- Main Program ---
def main():
    """Main program execution"""
    global current_robot_grid_pos_actual, current_robot_grid_pos_path
    global goal_grid_pos, planned_path, current_path_index
    global path_needs_replan, last_replan_time
    
    wlan = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    
    if not wlan or not wlan.isconnected():
        print("Stopping. No WiFi.")
        return
    
    server_socket = start_server(SERVER_PORT)
    conn = None  # Client connection

    while True:
        gc.collect()  # Free up memory
        current_time_ms = time.ticks_ms()

        # Wait for connection if not connected
        if conn is None:
            print("Waiting for Webots connection...")
            led.off()
            try:
                server_socket.settimeout(1.0)
                conn, addr = server_socket.accept()
                conn.settimeout(0.1)  # Non-blocking for recv
                print(f"Connected by Webots: {addr}")
                led.on()
                path_needs_replan = True
            except OSError as e:
                if e.args[0] == 116:  # ETIMEDOUT
                    pass
                else:
                    print(f"Accept error: {e}")
                    conn = None
                time.sleep(0.5)
                continue
            except Exception as e:
                print(f"Unexpected accept error: {e}")
                conn = None
                time.sleep(1)
                continue

        # Receive and process data from Webots
        try:
            data_bytes = conn.recv(512)
            if data_bytes:
                data_str = data_bytes.decode('utf-8').strip()
                led.value(not led.value())  # Blink on receive

                # Handle multiple JSON objects in stream
                for msg_part in data_str.split('\n'):
                    if not msg_part.strip():
                        continue
                    
                    try:
                        webots_data = json.loads(msg_part)
                        
                        if webots_data.get('type') == 'webots_status':
                            # Extract data from Webots
                            new_robot_pos_actual = tuple(webots_data.get('robot_grid_pos'))
                            new_goal_pos = tuple(webots_data.get('goal_grid_pos'))
                            world_pose = webots_data.get('world_pose', {})
                            robot_theta_rad = world_pose.get('theta_rad', 0.0)
                            line_sensors_binary = webots_data.get('sensors_binary', [0,0,0])
                            distance_sensor_value = webots_data.get('distance_sensor', 1.0)

                            # Update robot position
                            if new_robot_pos_actual != current_robot_grid_pos_actual:
                                current_robot_grid_pos_actual = new_robot_pos_actual
                                
                                # Check if robot deviated from expected path
                                if current_robot_grid_pos_path and \
                                   (abs(current_robot_grid_pos_actual[0] - current_robot_grid_pos_path[0]) > 1 or \
                                    abs(current_robot_grid_pos_actual[1] - current_robot_grid_pos_path[1]) > 1):
                                    print(f"Robot deviated from path. Forcing replan.")
                                    path_needs_replan = True
                                
                                if current_robot_grid_pos_path is None:
                                    current_robot_grid_pos_path = current_robot_grid_pos_actual

                            # Update goal if changed
                            if new_goal_pos != goal_grid_pos:
                                goal_grid_pos = new_goal_pos
                                path_needs_replan = True
                                print(f"New goal: {goal_grid_pos}")
                            
                            # Initialize positions if first update
                            if current_robot_grid_pos_actual is None:
                                current_robot_grid_pos_actual = new_robot_pos_actual
                            if current_robot_grid_pos_path is None:
                                current_robot_grid_pos_path = new_robot_pos_actual
                            if goal_grid_pos is None:
                                goal_grid_pos = new_goal_pos
                                path_needs_replan = True

                            # Distance sensor obstacle detection
                            if current_robot_grid_pos_actual and distance_sensor_value < 1.0:
                                sensor_obstacles = detect_obstacles_from_distance_sensor(
                                    distance_sensor_value, 
                                    current_robot_grid_pos_actual, 
                                    robot_theta_rad
                                )
                                if sensor_obstacles:
                                    path_needs_replan = True

                            # Path planning with A*
                            if path_needs_replan or (time.ticks_diff(current_time_ms, last_replan_time) > REPLAN_INTERVAL_MS):
                                if current_robot_grid_pos_actual and goal_grid_pos:
                                    print(f"Planning path: {current_robot_grid_pos_actual} -> {goal_grid_pos}")
                                    gc.collect()
                                    
                                    # Use A* algorithm with Manhattan heuristic
                                    new_path = a_star(grid_map, current_robot_grid_pos_actual, 
                                                     goal_grid_pos, heuristic='manhattan')
                                    gc.collect()
                                    
                                    if new_path:
                                        planned_path = new_path
                                        current_path_index = 0
                                        
                                        # Align robot position with path
                                        if planned_path[0] == current_robot_grid_pos_actual:
                                            current_robot_grid_pos_path = planned_path[0]
                                        else:
                                            print(f"WARN: Path starts at {planned_path[0]}, robot at {current_robot_grid_pos_actual}")
                                            try:
                                                current_path_index = planned_path.index(current_robot_grid_pos_actual)
                                                current_robot_grid_pos_path = current_robot_grid_pos_actual
                                                print(f"Robot found on path at index {current_path_index}")
                                            except ValueError:
                                                current_robot_grid_pos_path = planned_path[0]
                                                current_path_index = 0
                                        
                                        path_needs_replan = False
                                        last_replan_time = current_time_ms
                                    else:
                                        print("Failed to find path.")
                                        planned_path = []
                                        path_needs_replan = True
                                else:
                                    print("Cannot plan: position or goal unknown.")

                            # Determine action
                            action_to_send = 'stop'
                            if planned_path and current_robot_grid_pos_path and goal_grid_pos:
                                action_to_send, _ = get_action_from_path(
                                    current_robot_grid_pos_path,
                                    robot_theta_rad,
                                    line_sensors_binary
                                )
                                
                                # Update path progress
                                if current_robot_grid_pos_actual == current_robot_grid_pos_path:
                                    if action_to_send == 'forward' and current_path_index < len(planned_path) - 1:
                                        if current_path_index < len(planned_path) - 1:
                                            next_node = planned_path[current_path_index + 1]
                                            if current_robot_grid_pos_actual == next_node:
                                                current_path_index += 1
                                                current_robot_grid_pos_path = next_node
                                                print(f"Advanced to path index {current_path_index}")

                         

                            # Send command to Webots
                            command = {
                                'type': 'esp32_command',
                                'action': action_to_send,
                                'path': planned_path,
                                'robot_pos_on_path_esp_thinks': list(current_robot_grid_pos_path) if current_robot_grid_pos_path else None,
                                'current_path_idx_esp': current_path_index,
                                'algorithm': 'A*'  # Indicate we're using A*
                            }
                            response_json = json.dumps(command) + '\n'
                            conn.sendall(response_json.encode('utf-8'))

                    except json.JSONDecodeError as e:
                        print(f"JSON Error: {e}")
                    except Exception as e:
                        print(f"Processing error: {e}")
            else:
                # Empty data = disconnection
                print("Webots disconnected.")
                conn.close()
                conn = None
                led.off()
                # Reset state
                current_robot_grid_pos_actual = None
                current_robot_grid_pos_path = None
                planned_path = []
                path_needs_replan = True

        except OSError as e:
            if e.args[0] == 116:  # ETIMEDOUT
                pass
            elif e.args[0] == 104:  # ECONNRESET
                print("Connection reset by peer.")
                if conn:
                    conn.close()
                conn = None
                led.off()
            else:
                print(f"Socket error: {e}")
                if conn:
                    conn.close()
                conn = None
                led.off()
        except Exception as e:
            print(f"Unexpected error: {e}")
            if conn:
                conn.close()
            conn = None
            led.off()
            time.sleep(1)

        time.sleep(0.02)  # Small delay for stability

if __name__ == "__main__":
    main()