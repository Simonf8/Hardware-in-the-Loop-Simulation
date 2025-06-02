# ESP32 MicroPython Controller with D* Lite Algorithm for Webots HIL
# Advanced implementation for dynamic replanning

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
current_robot_grid_pos_actual = None
current_robot_grid_pos_path = None
goal_grid_pos = None
planned_path = []
current_path_index = 0
path_needs_replan = True
last_replan_time = 0
REPLAN_INTERVAL_MS = 2000

# --- D* Lite Algorithm Implementation ---
class DStarLite:
    """
    D* Lite algorithm implementation for dynamic pathfinding.
    Efficient for replanning when obstacles are detected.
    """
    
    def __init__(self, grid, start, goal):
        """
        Initialize D* Lite algorithm.
        
        Args:
            grid: 2D grid map (0 = free, 1 = obstacle)
            start: (row, col) starting position
            goal: (row, col) goal position
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.start = start
        self.goal = goal
        
        # D* Lite specific variables
        self.k_m = 0  # Key modifier for priority queue
        self.rhs = {}  # One-step lookahead values
        self.g = {}  # Cost values
        self.open_list = []  # Priority queue
        self.path = []
        
        # Initialize all cells
        for r in range(self.rows):
            for c in range(self.cols):
                self.rhs[(r, c)] = float('inf')
                self.g[(r, c)] = float('inf')
        
        # Goal has 0 cost
        self.rhs[self.goal] = 0
        self.open_list.append((self.calculate_key(self.goal), self.goal))
        
    def calculate_key(self, node):
        """
        Calculate priority key for a node.
        
        Args:
            node: (row, col) position
            
        Returns:
            (k1, k2) priority tuple
        """
        g_val = self.g.get(node, float('inf'))
        rhs_val = self.rhs.get(node, float('inf'))
        min_val = min(g_val, rhs_val)
        
        k1 = min_val + self.heuristic(self.start, node) + self.k_m
        k2 = min_val
        return (k1, k2)
    
    def heuristic(self, node1, node2):
        """
        Manhattan distance heuristic.
        
        Args:
            node1, node2: (row, col) positions
            
        Returns:
            Manhattan distance
        """
        return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])
    
    def get_neighbors(self, node):
        """
        Get valid neighbors of a node.
        
        Args:
            node: (row, col) position
            
        Returns:
            List of (neighbor, cost) tuples
        """
        r, c = node
        neighbors = []
        
        # 4-directional movement
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                # Cost is infinity for obstacles, 1 for free cells
                cost = float('inf') if self.grid[nr][nc] == 1 else 1
                neighbors.append(((nr, nc), cost))
        
        return neighbors
    
    def update_node(self, node):
        """
        Update a node's rhs value and queue status.
        
        Args:
            node: (row, col) position to update
        """
        if node != self.goal:
            # Calculate minimum rhs from successors
            min_rhs = float('inf')
            for neighbor, cost in self.get_neighbors(node):
                if cost < float('inf'):  # Skip obstacles
                    new_rhs = self.g.get(neighbor, float('inf')) + cost
                    min_rhs = min(min_rhs, new_rhs)
            self.rhs[node] = min_rhs
        
        # Remove from open list if present
        self.open_list = [(k, n) for k, n in self.open_list if n != node]
        
        # Add back if inconsistent
        if self.g.get(node, float('inf')) != self.rhs.get(node, float('inf')):
            self.open_list.append((self.calculate_key(node), node))
            self.open_list.sort()  # Keep sorted by key
    
    def compute_shortest_path(self):
        """
        Compute the shortest path using D* Lite algorithm.
        
        Returns:
            True if path exists, False otherwise
        """
        iterations = 0
        max_iterations = self.rows * self.cols * 4  # Prevent infinite loops
        
        while self.open_list and iterations < max_iterations:
            iterations += 1
            
            # Get node with minimum key
            if not self.open_list:
                break
                
            min_key, u = self.open_list[0]
            
            # Check termination condition
            start_key = self.calculate_key(self.start)
            if min_key >= start_key and self.rhs[self.start] == self.g[self.start]:
                break
            
            # Remove minimum from open list
            self.open_list.pop(0)
            
            # Update node
            if self.g[u] > self.rhs[u]:
                # Overconsistent - update g value
                self.g[u] = self.rhs[u]
                
                # Update all predecessors
                for neighbor, cost in self.get_neighbors(u):
                    if cost < float('inf'):  # Skip obstacles
                        self.update_node(neighbor)
            else:
                # Underconsistent - reset g value
                self.g[u] = float('inf')
                self.update_node(u)
                
                # Update all predecessors
                for neighbor, cost in self.get_neighbors(u):
                    if cost < float('inf'):
                        self.update_node(neighbor)
        
        # Check if path exists
        return self.g[self.start] < float('inf')
    
    def get_path(self):
        """
        Extract path from start to goal.
        
        Returns:
            List of (row, col) positions forming the path
        """
        if self.g[self.start] == float('inf'):
            return []  # No path exists
        
        path = []
        current = self.start
        
        # Greedy path extraction
        while current != self.goal:
            path.append(current)
            
            # Find best neighbor
            min_cost = float('inf')
            best_neighbor = None
            
            for neighbor, cost in self.get_neighbors(current):
                if cost < float('inf'):  # Skip obstacles
                    neighbor_cost = self.g.get(neighbor, float('inf')) + cost
                    if neighbor_cost < min_cost:
                        min_cost = neighbor_cost
                        best_neighbor = neighbor
            
            if best_neighbor is None:
                return []  # No valid path
                
            current = best_neighbor
            
            # Prevent infinite loops
            if len(path) > self.rows * self.cols:
                print("D* Lite: Path extraction failed (loop detected)")
                return []
        
        path.append(self.goal)
        return path
    
    def update_obstacle(self, position, is_obstacle):
        """
        Update grid when obstacle status changes.
        
        Args:
            position: (row, col) position that changed
            is_obstacle: True if position is now obstacle, False if free
        """
        r, c = position
        old_value = self.grid[r][c]
        new_value = 1 if is_obstacle else 0
        
        if old_value != new_value:
            self.grid[r][c] = new_value
            
            # Update affected nodes
            self.update_node(position)
            for neighbor, _ in self.get_neighbors(position):
                self.update_node(neighbor)
    
    def replan(self, new_start):
        """
        Replan from new start position (for dynamic replanning).
        
        Args:
            new_start: New starting position
            
        Returns:
            New path as list of (row, col) positions
        """
        if new_start != self.start:
            self.k_m += self.heuristic(self.start, new_start)
            self.start = new_start
        
        if self.compute_shortest_path():
            return self.get_path()
        return []

# Global D* Lite instance
dstar_planner = None

def init_dstar_planner(start, goal):
    """
    Initialize or reinitialize D* Lite planner.
    
    Args:
        start: Starting position
        goal: Goal position
    """
    global dstar_planner
    dstar_planner = DStarLite(
        [[cell for cell in row] for row in grid_map],  # Copy grid
        start, 
        goal
    )
    dstar_planner.compute_shortest_path()

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
        timeout = 10
        
        while not wlan.isconnected() and timeout > 0:
            print('.', end='')
            led.value(not led.value())
            time.sleep(1)
            timeout -= 1
    
    if wlan.isconnected():
        led.on()
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

# --- Path Following Logic ---
def get_action_from_path(robot_pos_on_path, world_theta_rad, webots_line_sensors_binary):
    """
    Determine robot action based on current position and planned path.
    
    Args:
        robot_pos_on_path: Current robot position on path
        world_theta_rad: Robot's current orientation in radians
        webots_line_sensors_binary: Line sensor readings
    
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
        print(f"WARN: Path index {current_path_index} out of bounds")
        return 'stop', robot_pos_on_path

    current_node_on_path = planned_path[current_path_index]
    next_node_on_path = planned_path[current_path_index + 1]

    # Re-align path index if needed
    if robot_pos_on_path != current_node_on_path:
        print(f"WARN: Re-aligning path index")
        try:
            current_path_index = planned_path.index(robot_pos_on_path, current_path_index)
            current_node_on_path = planned_path[current_path_index]
            if current_path_index >= len(planned_path) - 1:
                return 'stop', robot_pos_on_path
            next_node_on_path = planned_path[current_path_index + 1]
        except ValueError:
            print(f"ERROR: Robot position not found in path")
            return 'stop', robot_pos_on_path

    # Determine target orientation
    dr = next_node_on_path[0] - current_node_on_path[0]
    dc = next_node_on_path[1] - current_node_on_path[1]

    target_theta_rad = None
    if dc == 1 and dr == 0:
        target_theta_rad = 0.0  # Right
    elif dc == -1 and dr == 0:
        target_theta_rad = math.pi  # Left
    elif dr == 1 and dc == 0:
        target_theta_rad = math.pi / 2.0  # Down
    elif dr == -1 and dc == 0:
        target_theta_rad = -math.pi / 2.0  # Up
    else:
        print(f"WARN: Non-adjacent nodes")
        return 'stop', current_node_on_path

    # Calculate angle difference
    current_theta_norm = math.atan2(math.sin(world_theta_rad), math.cos(world_theta_rad))
    angle_diff = target_theta_rad - current_theta_norm
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    ANGLE_THRESHOLD_RAD = math.radians(40)

    if abs(angle_diff) > ANGLE_THRESHOLD_RAD:
        return 'turn_left' if angle_diff > 0 else 'turn_right', current_node_on_path
    else:
        return 'forward', current_node_on_path

# --- Obstacle Detection Handler ---
def handle_obstacle_detection(detected_obstacles):
    """
    Handle newly detected obstacles and trigger replanning.
    
    Args:
        detected_obstacles: List of (row, col) positions with obstacles
    """
    global dstar_planner, path_needs_replan
    
    if not detected_obstacles:
        return
    
    if dstar_planner:
        obstacle_count = 0
        for obs_pos in detected_obstacles:
            # Convert to tuple if it's a list
            if isinstance(obs_pos, list):
                obs_pos = tuple(obs_pos)
            
            # Validate obstacle position
            if (isinstance(obs_pos, tuple) and len(obs_pos) == 2 and
                0 <= obs_pos[0] < GRID_ROWS and 0 <= obs_pos[1] < GRID_COLS):
                print(f"D* Lite: Updating obstacle at {obs_pos}")
                dstar_planner.update_obstacle(obs_pos, True)
                obstacle_count += 1
            else:
                print(f"D* Lite: Invalid obstacle position: {obs_pos}")
        
        if obstacle_count > 0:
            path_needs_replan = True
            print(f"D* Lite: {obstacle_count} obstacles processed, replanning required")
    else:
        print(f"D* Lite: Cannot process obstacles - planner not initialized")

# --- Main Program ---
def main():
    """Main program execution"""
    global current_robot_grid_pos_actual, current_robot_grid_pos_path
    global goal_grid_pos, planned_path, current_path_index
    global path_needs_replan, last_replan_time, dstar_planner
    
    wlan = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    
    if not wlan or not wlan.isconnected():
        print("Stopping. No WiFi.")
        return
    
    server_socket = start_server(SERVER_PORT)
    conn = None

    while True:
        gc.collect()
        current_time_ms = time.ticks_ms()

        # Wait for connection
        if conn is None:
            print("Waiting for Webots connection...")
            led.off()
            try:
                server_socket.settimeout(1.0)
                conn, addr = server_socket.accept()
                conn.settimeout(0.1)
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
                print(f"Unexpected error: {e}")
                conn = None
                time.sleep(1)
                continue

        # Receive and process data
        try:
            data_bytes = conn.recv(512)
            if data_bytes:
                data_str = data_bytes.decode('utf-8').strip()
                led.value(not led.value())

                for msg_part in data_str.split('\n'):
                    if not msg_part.strip():
                        continue
                    
                    try:
                        webots_data = json.loads(msg_part)
                        
                        if webots_data.get('type') == 'webots_status':
                            # Extract data
                            new_robot_pos_actual = tuple(webots_data.get('robot_grid_pos'))
                            new_goal_pos = tuple(webots_data.get('goal_grid_pos'))
                            world_pose = webots_data.get('world_pose', {})
                            robot_theta_rad = world_pose.get('theta_rad', 0.0)
                            line_sensors_binary = webots_data.get('sensors_binary', [0,0,0])
                            detected_obstacles = webots_data.get('detected_obstacles', [])
                            
                            # Debug obstacle reception
                            if detected_obstacles:
                                print(f"📨 Received {len(detected_obstacles)} obstacles from Webots: {detected_obstacles}")

                            # Update robot position
                            if new_robot_pos_actual != current_robot_grid_pos_actual:
                                current_robot_grid_pos_actual = new_robot_pos_actual
                                
                                if current_robot_grid_pos_path and \
                                   (abs(current_robot_grid_pos_actual[0] - current_robot_grid_pos_path[0]) > 1 or \
                                    abs(current_robot_grid_pos_actual[1] - current_robot_grid_pos_path[1]) > 1):
                                    print(f"Robot deviated significantly")
                                    path_needs_replan = True
                                
                                if current_robot_grid_pos_path is None:
                                    current_robot_grid_pos_path = current_robot_grid_pos_actual

                            # Update goal
                            if new_goal_pos != goal_grid_pos:
                                goal_grid_pos = new_goal_pos
                                path_needs_replan = True
                                print(f"New goal: {goal_grid_pos}")
                                # Reinitialize D* Lite with new goal
                                if current_robot_grid_pos_actual:
                                    init_dstar_planner(current_robot_grid_pos_actual, goal_grid_pos)

                            # Initialize positions
                            if current_robot_grid_pos_actual is None:
                                current_robot_grid_pos_actual = new_robot_pos_actual
                            if current_robot_grid_pos_path is None:
                                current_robot_grid_pos_path = new_robot_pos_actual
                            if goal_grid_pos is None:
                                goal_grid_pos = new_goal_pos
                                path_needs_replan = True

                            # Handle detected obstacles with D* Lite
                            if detected_obstacles:
                                print(f"📍 Processing obstacles: {detected_obstacles}")
                                handle_obstacle_detection(detected_obstacles)
                            elif dstar_planner:
                                # Only print this occasionally to avoid spam
                                if current_time_ms % 5000 < 100:  # Every 5 seconds, for 100ms window
                                    print(f"📡 No obstacles in current message")

                            # Path planning with D* Lite
                            if path_needs_replan or (time.ticks_diff(current_time_ms, last_replan_time) > REPLAN_INTERVAL_MS):
                                if current_robot_grid_pos_actual and goal_grid_pos:
                                    print(f"D* Lite planning: {current_robot_grid_pos_actual} -> {goal_grid_pos}")
                                    gc.collect()
                                    
                                    # Initialize D* Lite if needed
                                    if dstar_planner is None:
                                        init_dstar_planner(current_robot_grid_pos_actual, goal_grid_pos)
                                    
                                    # Replan from current position
                                    new_path = dstar_planner.replan(current_robot_grid_pos_actual)
                                    gc.collect()
                                    
                                    if new_path:
                                        planned_path = new_path
                                        current_path_index = 0
                                        
                                        if planned_path[0] == current_robot_grid_pos_actual:
                                            current_robot_grid_pos_path = planned_path[0]
                                        else:
                                            print(f"WARN: Path alignment issue")
                                            try:
                                                current_path_index = planned_path.index(current_robot_grid_pos_actual)
                                                current_robot_grid_pos_path = current_robot_grid_pos_actual
                                            except ValueError:
                                                current_robot_grid_pos_path = planned_path[0]
                                                current_path_index = 0
                                        
                                        path_needs_replan = False
                                        last_replan_time = current_time_ms
                                        print(f"D* Lite: Path with {len(planned_path)} steps")
                                    else:
                                        print("D* Lite: No path found")
                                        planned_path = []
                                        path_needs_replan = True

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
                                        next_node = planned_path[current_path_index + 1]
                                        if current_robot_grid_pos_actual == next_node:
                                            current_path_index += 1
                                            current_robot_grid_pos_path = next_node
                                            print(f"D* Lite: Advanced to index {current_path_index}")

                            # Check goal reached
                            if current_robot_grid_pos_actual == goal_grid_pos:
                                action_to_send = 'stop'
                                print("🎉 Goal Reached with D* Lite!")
                                planned_path = []
                                path_needs_replan = False

                            # Send command
                            command = {
                                'type': 'esp32_command',
                                'action': action_to_send,
                                'path': planned_path,
                                'robot_pos_on_path_esp_thinks': list(current_robot_grid_pos_path) if current_robot_grid_pos_path else None,
                                'current_path_idx_esp': current_path_index,
                                'algorithm': 'D* Lite'
                            }
                            response_json = json.dumps(command) + '\n'
                            conn.sendall(response_json.encode('utf-8'))

                    except ValueError as e:
                        print(f"JSON Error: {e}")
                    except Exception as e:
                        print(f"Processing error: {e}")
            else:
                # Disconnection
                print("Webots disconnected")
                conn.close()
                conn = None
                led.off()
                # Reset state
                current_robot_grid_pos_actual = None
                current_robot_grid_pos_path = None
                planned_path = []
                path_needs_replan = True
                dstar_planner = None

        except OSError as e:
            if e.args[0] == 116:  # ETIMEDOUT
                pass
            elif e.args[0] == 104:  # ECONNRESET
                print("Connection reset")
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

        time.sleep(0.02)

if __name__ == "__main__":
    main()
