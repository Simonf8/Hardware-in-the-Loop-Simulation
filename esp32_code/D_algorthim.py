# Simon.(CJ)
# Hardware-in-the-Loop ESP32 Controller with D* Lite Algorithm
# Advanced implementation for dynamic path replanning in robotic navigation

import network
import socket
import json
import time
import gc
from machine import Pin
import math

# Network Configuration
WIFI_SSID = 'CJ'
WIFI_PASSWORD = '4533simon'
SERVER_PORT = 8080

# Hardware Configuration
onboard_led = Pin(2, Pin.OUT)

# Navigation Configuration
GRID_ROWS = 15
GRID_COLS = 21
REPLAN_INTERVAL_MS = 2000

# Grid map representation (0 = pathable, 1 = obstacle)
static_grid_map = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]


class DStarLitePathPlanner:
    """
    D* Lite algorithm implementation for dynamic pathfinding in robotics.
    Efficient replanning when obstacles are detected during navigation.
    """
    
    def __init__(self, grid, start_position, goal_position):
        """Initialize D* Lite planner with grid and positions."""
        self.grid = [row[:] for row in grid]  # Create deep copy
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.start = start_position
        self.goal = goal_position
        
        # D* Lite algorithm state variables
        self.key_modifier = 0
        self.rhs_values = {}
        self.g_values = {}
        self.priority_queue = []
        
        self._initialize_algorithm()
    
    def _initialize_algorithm(self):
        """Initialize all grid cells with infinite cost except goal."""
        for row in range(self.rows):
            for col in range(self.cols):
                position = (row, col)
                self.rhs_values[position] = float('inf')
                self.g_values[position] = float('inf')
        
        self.rhs_values[self.goal] = 0
        self.priority_queue = [(self._calculate_priority_key(self.goal), self.goal)]
    
    def _calculate_priority_key(self, position):
        """Calculate priority key for queue ordering."""
        g_val = self.g_values.get(position, float('inf'))
        rhs_val = self.rhs_values.get(position, float('inf'))
        min_val = min(g_val, rhs_val)
        
        primary_key = min_val + self._heuristic_distance(self.start, position) + self.key_modifier
        secondary_key = min_val
        return (primary_key, secondary_key)
    
    def _heuristic_distance(self, pos1, pos2):
        """Manhattan distance heuristic for pathfinding."""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def _get_valid_neighbors(self, position):
        """Get valid neighboring cells with movement costs."""
        row, col = position
        neighbors = []
        
        # Check 4-directional movement
        for delta_row, delta_col in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_row, new_col = row + delta_row, col + delta_col
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                neighbor_pos = (new_row, new_col)
                movement_cost = float('inf') if self.grid[new_row][new_col] == 1 else 1
                neighbors.append((neighbor_pos, movement_cost))
        
        return neighbors
    
    def _update_node_consistency(self, position):
        """Update node's consistency and priority queue status."""
        if position != self.goal:
            min_rhs = float('inf')
            for neighbor, cost in self._get_valid_neighbors(position):
                if cost < float('inf'):
                    neighbor_cost = self.g_values.get(neighbor, float('inf')) + cost
                    min_rhs = min(min_rhs, neighbor_cost)
            self.rhs_values[position] = min_rhs
        
        # Remove from queue if present
        self.priority_queue = [(key, pos) for key, pos in self.priority_queue if pos != position]
        
        # Add back if inconsistent
        if self.g_values.get(position, float('inf')) != self.rhs_values.get(position, float('inf')):
            self.priority_queue.append((self._calculate_priority_key(position), position))
            self.priority_queue.sort()
    
    def _compute_shortest_path(self):
        """Execute D* Lite algorithm to find shortest path."""
        max_iterations = self.rows * self.cols * 2
        iteration_count = 0
        
        while self.priority_queue and iteration_count < max_iterations:
            iteration_count += 1
            
            min_key, current_node = self.priority_queue[0]
            start_key = self._calculate_priority_key(self.start)
            
            if (min_key >= start_key and 
                self.rhs_values[self.start] == self.g_values[self.start]):
                break
            
            self.priority_queue.pop(0)
            
            if self.g_values[current_node] > self.rhs_values[current_node]:
                # Overconsistent case
                self.g_values[current_node] = self.rhs_values[current_node]
                for neighbor, cost in self._get_valid_neighbors(current_node):
                    if cost < float('inf'):
                        self._update_node_consistency(neighbor)
            else:
                # Underconsistent case
                self.g_values[current_node] = float('inf')
                self._update_node_consistency(current_node)
                for neighbor, cost in self._get_valid_neighbors(current_node):
                    if cost < float('inf'):
                        self._update_node_consistency(neighbor)
        
        return self.g_values[self.start] < float('inf')
    
    def extract_path(self):
        """Extract path from start to goal using computed values."""
        if self.g_values[self.start] == float('inf'):
            return []
        
        path = []
        current = self.start
        
        while current != self.goal:
            path.append(current)
            
            best_neighbor = None
            min_cost = float('inf')
            
            for neighbor, movement_cost in self._get_valid_neighbors(current):
                if movement_cost < float('inf'):
                    total_cost = self.g_values.get(neighbor, float('inf')) + movement_cost
                    if total_cost < min_cost:
                        min_cost = total_cost
                        best_neighbor = neighbor
            
            if best_neighbor is None:
                return []
            
            current = best_neighbor
            
            # Prevent infinite loops
            if len(path) > self.rows * self.cols:
                return []
        
        path.append(self.goal)
        return path
    
    def update_obstacle_status(self, position, is_obstacle):
        """Update grid when obstacle status changes."""
        row, col = position
        if (0 <= row < self.rows and 0 <= col < self.cols):
            old_value = self.grid[row][col]
            new_value = 1 if is_obstacle else 0
            
            if old_value != new_value:
                self.grid[row][col] = new_value
                self._update_node_consistency(position)
                for neighbor, _ in self._get_valid_neighbors(position):
                    self._update_node_consistency(neighbor)
    
    def replan_from_position(self, new_start):
        """Replan path from new starting position."""
        if new_start != self.start:
            self.key_modifier += self._heuristic_distance(self.start, new_start)
            self.start = new_start
        
        if self._compute_shortest_path():
            return self.extract_path()
        return []


class NavigationController:
    """Main navigation controller managing robot movement and path following."""
    
    def __init__(self):
        self.current_position = None
        self.goal_position = None
        self.planned_path = []
        self.path_index = 0
        self.needs_replanning = True
        self.last_replan_time = 0
        self.path_planner = None
    
    def initialize_planner(self, start_pos, goal_pos):
        """Initialize or reinitialize the path planner."""
        self.path_planner = DStarLitePathPlanner(static_grid_map, start_pos, goal_pos)
        self.path_planner._compute_shortest_path()
    
    def process_obstacle_updates(self, detected_obstacles):
        """Process newly detected obstacles and trigger replanning."""
        if not detected_obstacles or not self.path_planner:
            return
        
        obstacle_count = 0
        for obstacle_pos in detected_obstacles:
            if isinstance(obstacle_pos, list):
                obstacle_pos = tuple(obstacle_pos)
            
            if (isinstance(obstacle_pos, tuple) and len(obstacle_pos) == 2 and
                0 <= obstacle_pos[0] < GRID_ROWS and 0 <= obstacle_pos[1] < GRID_COLS):
                self.path_planner.update_obstacle_status(obstacle_pos, True)
                obstacle_count += 1
        
        if obstacle_count > 0:
            self.needs_replanning = True
    
    def calculate_movement_action(self, robot_position, robot_orientation, line_sensors):
        """Determine robot action based on current position and planned path."""
        if not self.planned_path or not robot_position:
            return 'stop'
        
        if robot_position == self.goal_position:
            return 'stop'
        
        # Ensure valid path index
        if not (0 <= self.path_index < len(self.planned_path) - 1):
            return 'stop'
        
        current_waypoint = self.planned_path[self.path_index]
        next_waypoint = self.planned_path[self.path_index + 1]
        
        # Realign path index if robot position doesn't match
        if robot_position != current_waypoint:
            try:
                self.path_index = self.planned_path.index(robot_position, self.path_index)
                current_waypoint = self.planned_path[self.path_index]
                if self.path_index >= len(self.planned_path) - 1:
                    return 'stop'
                next_waypoint = self.planned_path[self.path_index + 1]
            except ValueError:
                return 'stop'
        
        # Calculate required orientation
        delta_row = next_waypoint[0] - current_waypoint[0]
        delta_col = next_waypoint[1] - current_waypoint[1]
        
        target_orientation = None
        if delta_col == 1 and delta_row == 0:
            target_orientation = 0.0  # Right
        elif delta_col == -1 and delta_row == 0:
            target_orientation = math.pi  # Left
        elif delta_row == 1 and delta_col == 0:
            target_orientation = math.pi / 2.0  # Down
        elif delta_row == -1 and delta_col == 0:
            target_orientation = -math.pi / 2.0  # Up
        else:
            return 'stop'
        
        # Calculate orientation difference
        normalized_orientation = math.atan2(math.sin(robot_orientation), math.cos(robot_orientation))
        angle_difference = target_orientation - normalized_orientation
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))
        
        ORIENTATION_THRESHOLD = math.radians(40)
        
        if abs(angle_difference) > ORIENTATION_THRESHOLD:
            return 'turn_left' if angle_difference > 0 else 'turn_right'
        else:
            return 'forward'


def establish_wifi_connection(ssid, password):
    """Establish WiFi connection with network."""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        wlan.connect(ssid, password)
        timeout_counter = 10
        
        while not wlan.isconnected() and timeout_counter > 0:
            onboard_led.value(not onboard_led.value())
            time.sleep(1)
            timeout_counter -= 1
    
    if wlan.isconnected():
        onboard_led.on()
        print(f'WiFi Connected: {wlan.ifconfig()[0]}')
        return wlan
    else:
        onboard_led.off()
        return None


def create_server_socket(port):
    """Create and configure TCP server socket."""
    address = socket.getaddrinfo('0.0.0.0', port)[0][-1]
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(address)
    server_socket.listen(1)
    return server_socket


def main():
    """Main program execution loop."""
    # Initialize navigation controller
    nav_controller = NavigationController()
    
    # Establish network connection
    wifi_connection = establish_wifi_connection(WIFI_SSID, WIFI_PASSWORD)
    if not wifi_connection:
        return
    
    # Create server
    server_socket = create_server_socket(SERVER_PORT)
    print(f'D* Lite Navigation Server Active on Port {SERVER_PORT}')
    client_connection = None
    
    while True:
        gc.collect()
        current_time_ms = time.ticks_ms()
        
        # Handle client connection
        if client_connection is None:
            try:
                server_socket.settimeout(1.0)
                client_connection, client_address = server_socket.accept()
                client_connection.settimeout(0.1)
                onboard_led.on()
                nav_controller.needs_replanning = True
            except OSError as e:
                if e.args[0] != 116:  # Not timeout
                    client_connection = None
                time.sleep(0.5)
                continue
            except Exception:
                client_connection = None
                time.sleep(1)
                continue
        
        # Process incoming data
        try:
            received_data = client_connection.recv(512)
            if received_data:
                data_string = received_data.decode('utf-8').strip()
                onboard_led.value(not onboard_led.value())
                
                for message_part in data_string.split('\n'):
                    if not message_part.strip():
                        continue
                    
                    try:
                        webots_data = json.loads(message_part)
                        
                        if webots_data.get('type') == 'webots_status':
                            # Extract navigation data
                            robot_position = tuple(webots_data.get('robot_grid_pos'))
                            goal_position = tuple(webots_data.get('goal_grid_pos'))
                            world_pose = webots_data.get('world_pose', {})
                            robot_orientation = world_pose.get('theta_rad', 0.0)
                            line_sensors = webots_data.get('sensors_binary', [0,0,0])
                            detected_obstacles = webots_data.get('detected_obstacles', [])
                            
                            # Update controller state
                            if robot_position != nav_controller.current_position:
                                nav_controller.current_position = robot_position
                            
                            if goal_position != nav_controller.goal_position:
                                nav_controller.goal_position = goal_position
                                nav_controller.needs_replanning = True
                                if nav_controller.current_position:
                                    nav_controller.initialize_planner(nav_controller.current_position, goal_position)
                            
                            # Process obstacle updates
                            if detected_obstacles:
                                nav_controller.process_obstacle_updates(detected_obstacles)
                            
                            # Path planning and replanning
                            time_since_replan = time.ticks_diff(current_time_ms, nav_controller.last_replan_time)
                            if (nav_controller.needs_replanning or time_since_replan > REPLAN_INTERVAL_MS):
                                if nav_controller.current_position and nav_controller.goal_position:
                                    gc.collect()
                                    
                                    if nav_controller.path_planner is None:
                                        nav_controller.initialize_planner(nav_controller.current_position, nav_controller.goal_position)
                                    
                                    new_path = nav_controller.path_planner.replan_from_position(nav_controller.current_position)
                                    gc.collect()
                                    
                                    if new_path:
                                        nav_controller.planned_path = new_path
                                        nav_controller.path_index = 0
                                        nav_controller.needs_replanning = False
                                        nav_controller.last_replan_time = current_time_ms
                                    else:
                                        nav_controller.planned_path = []
                                        nav_controller.needs_replanning = True
                            
                            # Determine movement action
                            movement_action = 'stop'
                            if nav_controller.planned_path and nav_controller.current_position and nav_controller.goal_position:
                                movement_action = nav_controller.calculate_movement_action(
                                    nav_controller.current_position,
                                    robot_orientation,
                                    line_sensors
                                )
                                
                                # Update path progress
                                if (movement_action == 'forward' and 
                                    nav_controller.path_index < len(nav_controller.planned_path) - 1):
                                    next_waypoint = nav_controller.planned_path[nav_controller.path_index + 1]
                                    if nav_controller.current_position == next_waypoint:
                                        nav_controller.path_index += 1
                            
                            # Check goal completion
                            if nav_controller.current_position == nav_controller.goal_position:
                                movement_action = 'stop'
                                nav_controller.planned_path = []
                                nav_controller.needs_replanning = False
                            
                            # Send response
                            response_data = {
                                'type': 'esp32_command',
                                'action': movement_action,
                                'path': nav_controller.planned_path,
                                'robot_pos_on_path_esp_thinks': list(nav_controller.current_position) if nav_controller.current_position else None,
                                'current_path_idx_esp': nav_controller.path_index,
                                'algorithm': 'D* Lite'
                            }
                            response_json = json.dumps(response_data) + '\n'
                            client_connection.sendall(response_json.encode('utf-8'))
                    
                    except ValueError:
                        pass
                    except Exception:
                        pass
            else:
                # Client disconnected
                client_connection.close()
                client_connection = None
                onboard_led.off()
                # Reset navigation state
                nav_controller.current_position = None
                nav_controller.planned_path = []
                nav_controller.needs_replanning = True
                nav_controller.path_planner = None
        
        except OSError as e:
            if e.args[0] == 116:  # Timeout
                pass
            elif e.args[0] == 104:  # Connection reset
                if client_connection:
                    client_connection.close()
                client_connection = None
                onboard_led.off()
            else:
                if client_connection:
                    client_connection.close()
                client_connection = None
                onboard_led.off()
        except Exception:
            if client_connection:
                client_connection.close()
            client_connection = None
            onboard_led.off()
            time.sleep(1)
        
        time.sleep(0.02)


if __name__ == "__main__":
    main()