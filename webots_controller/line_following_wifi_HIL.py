"""
Simon(CJ), Heba
Hardware-in-the-Loop Webots Controller with ESP32 Integration
Real-time path planning with sensor-based visualization and obstacle detection
"""
from controller import Robot, DistanceSensor, Motor
import socket
import time
import math
import matplotlib.pyplot as plt
import json

# Network Configuration
ESP32_IP_ADDRESS = "192.168.53.245"
ESP32_PORT = 8080

# Robot Physical Parameters
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.05810

# Grid Configuration
GRID_ROWS = 15
GRID_COLS = 21
GRID_CELL_SIZE = 0.05099
GRID_ORIGIN_X = 0.050002
GRID_ORIGIN_Z = -0.639e-05

# Navigation Parameters
GOAL_ROW = 14
GOAL_COL = 0
FORWARD_SPEED = 1.5
LINE_THRESHOLD = 600

# Obstacle Detection Configuration
DISTANCE_SENSOR_THRESHOLD = 110
OBSTACLE_DETECTION_ENABLED = True

# Movement Control Parameters
TURN_SPEED_FACTOR = 1.5
MIN_INITIAL_SPIN_DURATION = 0.6
MAX_SEARCH_SPIN_DURATION = 18.9
MAX_ADJUST_DURATION = 2.20
TURN_ADJUST_BASE_SPEED = FORWARD_SPEED * 0.8
TURN_UNTIL_LINE_FOUND = True

# Line Following Parameters
AGGRESSIVE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 1.3
MODERATE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 1.2

# Starting Position Configuration
INITIAL_GRID_ROW = 2
INITIAL_GRID_COL = 20

# World grid definition (0 = Black Line, 1 = White Space)
world_grid = [
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


class CoordinateConverter:
    """Handle conversion between world and grid coordinates."""
    
    @staticmethod
    def world_to_grid(world_x, world_z):
        """Convert world coordinates to grid coordinates."""
        col = round((world_x - GRID_ORIGIN_X) / GRID_CELL_SIZE)
        row = round((world_z - GRID_ORIGIN_Z) / GRID_CELL_SIZE)
        col = max(0, min(col, GRID_COLS - 1))
        row = max(0, min(row, GRID_ROWS - 1))
        return row, col
    
    @staticmethod
    def grid_to_world_center(row, col):
        """Convert grid coordinates to world coordinates (center of cell)."""
        world_x = GRID_ORIGIN_X + col * GRID_CELL_SIZE
        world_z = GRID_ORIGIN_Z + row * GRID_CELL_SIZE
        return world_x, world_z

#1
class ObstacleDetector:
    """Handle obstacle detection using distance sensors."""
    
    def __init__(self):
        self.detected_obstacles = set()
        self.recent_obstacles = []
    
    def process_sensor_readings(self, robot_world_pose, robot_orientation, distance_values):
        """Process distance sensor readings and detect obstacles."""
        if not OBSTACLE_DETECTION_ENABLED:
            return []
        
        new_obstacles = []
        current_row, current_col = CoordinateConverter.world_to_grid(
            robot_world_pose['x'], robot_world_pose['z'])
        
        # Process three sensors: front, front-left, front-right
        for sensor_index, distance_value in enumerate(distance_values):
            if distance_value > DISTANCE_SENSOR_THRESHOLD:
                obstacle_position = self._calculate_obstacle_position(
                    current_row, current_col, robot_orientation, sensor_index)
                
                if self._is_valid_position(obstacle_position):
                    if obstacle_position not in self.detected_obstacles:
                        new_obstacles.append(obstacle_position)
                        self.detected_obstacles.add(obstacle_position)
        
        return new_obstacles
    
    def _calculate_obstacle_position(self, robot_row, robot_col, orientation, sensor_index):
        """Calculate obstacle position based on robot orientation and sensor."""
        theta_degrees = math.degrees(orientation) % 360
        
        # Determine robot facing direction
        if -45 <= theta_degrees <= 45 or 315 <= theta_degrees <= 360:
            # Robot facing RIGHT
            offsets = [(0, 1), (-1, 1), (1, 1)]  # front, front-left, front-right
        elif 45 < theta_degrees <= 135:
            # Robot facing DOWN
            offsets = [(1, 0), (1, 1), (1, -1)]
        elif 135 < theta_degrees <= 225:
            # Robot facing LEFT
            offsets = [(0, -1), (1, -1), (-1, -1)]
        else:
            # Robot facing UP
            offsets = [(-1, 0), (-1, -1), (-1, 1)]
        
        delta_row, delta_col = offsets[sensor_index]
        return (robot_row + delta_row, robot_col + delta_col)
    
    def _is_valid_position(self, position):
        """Check if position is within grid bounds."""
        row, col = position
        return 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS
    
    def get_recent_obstacles(self):
        """Get and clear recent obstacles for transmission."""
        obstacles = self.recent_obstacles.copy()
        self.recent_obstacles.clear()
        return obstacles
    
    def add_recent_obstacles(self, obstacles):
        """Add obstacles to recent list for transmission."""
        self.recent_obstacles.extend(obstacles)


class VisualizationManager:
    """Manage real-time visualization of navigation system."""
    
    def __init__(self):
        self.figure = None
        self.axis = None
        self.robot_trail = []
        self.planned_path = []
        self.obstacle_detector = ObstacleDetector()
    
    def initialize_display(self):
        """Initialize matplotlib visualization."""
        plt.ion()
        self.figure, self.axis = plt.subplots(figsize=(12, 9))
        self.axis.set_aspect('equal')
        self.axis.set_title('Hardware-in-the-Loop Navigation System', fontsize=14, fontweight='bold')
        self.axis.set_xlabel('World X (m)')
        self.axis.set_ylabel('World Z (m)')
        
        self._draw_grid_lines()
        self._setup_display_limits()
        self._create_legend()
        
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.01)
    
    def _draw_grid_lines(self):
        """Draw grid overlay on visualization."""
        # Horizontal lines
        for row in range(GRID_ROWS + 1):
            z_coord = GRID_ORIGIN_Z + row * GRID_CELL_SIZE
            self.axis.plot([GRID_ORIGIN_X, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE], 
                          [z_coord, z_coord], 'k-', alpha=0.2, lw=0.5)
        
        # Vertical lines
        for col in range(GRID_COLS + 1):
            x_coord = GRID_ORIGIN_X + col * GRID_CELL_SIZE
            self.axis.plot([x_coord, x_coord], 
                          [GRID_ORIGIN_Z, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE], 
                          'k-', alpha=0.2, lw=0.5)
    
    def _setup_display_limits(self):
        """Set appropriate display limits with margin."""
        margin = GRID_CELL_SIZE * 2
        self.axis.set_xlim(GRID_ORIGIN_X - margin, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE + margin)
        self.axis.set_ylim(GRID_ORIGIN_Z - margin, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE + margin)
    
    def _create_legend(self):
        """Create legend for visualization elements."""
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(fc='black', alpha=0.7, label='Black Line Path'),
            Patch(fc='lightgrey', alpha=0.3, label='White Space'),
            Patch(fc='red', alpha=0.7, label='Detected Obstacle'),
            plt.Line2D([0], [0], color='cyan', lw=2, label='Robot Trail'),
            plt.Line2D([0], [0], color='magenta', marker='o', ms=5, ls='--', lw=2, label='Planned Path'),
            plt.Line2D([0], [0], color='red', marker='o', ms=8, ls='', label='Robot Position'),
            plt.Line2D([0], [0], color='green', marker='*', ms=12, ls='', label='Goal Position')
        ]
        self.axis.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1))
    
    def update_display(self, robot_world_pose, robot_grid_position, line_sensors, planned_path):
        """Update visualization with current system state."""
        if self.figure is None:
            self.initialize_display()
        
        self._clear_dynamic_elements()
        self._draw_grid_cells()
        self._update_robot_trail(robot_world_pose)
        self._draw_planned_path(planned_path)
        self._draw_robot_state(robot_world_pose, robot_grid_position, line_sensors)
        self._draw_goal_position()
        self._display_system_info(robot_grid_position, line_sensors)
        
        plt.draw()
        plt.pause(0.001)
    
    def _clear_dynamic_elements(self):
        """Clear dynamic visualization elements for redraw."""
        num_static_lines = (GRID_ROWS + 1) + (GRID_COLS + 1)
        
        # Clear patches and texts
        for patch in self.axis.patches[:]:
            patch.remove()
        
        texts_to_remove = [t for t in self.axis.texts 
                          if t.get_position()[0] > GRID_ORIGIN_X - GRID_CELL_SIZE]
        for text in texts_to_remove:
            text.remove()
        
        # Clear dynamic lines
        lines_to_remove = self.axis.lines[num_static_lines:]
        for line in lines_to_remove:
            line.remove()
    
    def _draw_grid_cells(self):
        """Draw grid cells with appropriate coloring."""
        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                center_x, center_z = CoordinateConverter.grid_to_world_center(row, col)
                
                # Determine cell color based on obstacle status
                if (row, col) in self.obstacle_detector.detected_obstacles:
                    color, alpha = 'red', 0.7
                else:
                    color = 'black' if world_grid[row][col] == 0 else 'lightgrey'
                    alpha = 0.6 if color == 'black' else 0.3
                
                rect = plt.Rectangle(
                    (center_x - GRID_CELL_SIZE/2, center_z - GRID_CELL_SIZE/2),
                    GRID_CELL_SIZE, GRID_CELL_SIZE,
                    facecolor=color, alpha=alpha, edgecolor='gray', linewidth=0.5
                )
                self.axis.add_patch(rect)
    
    def _update_robot_trail(self, robot_world_pose):
        """Update and display robot movement trail."""
        self.robot_trail.append((robot_world_pose['x'], robot_world_pose['z']))
        if len(self.robot_trail) > 200:
            self.robot_trail.pop(0)
        
        if len(self.robot_trail) > 1:
            trail_x, trail_z = zip(*self.robot_trail)
            self.axis.plot(trail_x, trail_z, 'cyan', lw=2, alpha=0.7)
    
    def _draw_planned_path(self, planned_path):
        """Display planned path if available."""
        if planned_path and len(planned_path) > 1:
            self.planned_path = planned_path
            path_world = [CoordinateConverter.grid_to_world_center(r, c) for r, c in planned_path]
            if path_world:
                path_x, path_z = zip(*path_world)
                self.axis.plot(path_x, path_z, 'mo--', lw=2, ms=5, alpha=0.8)
    
    def _draw_robot_state(self, robot_world_pose, robot_grid_position, line_sensors):
        """Draw robot position and orientation."""
        # Robot position
        self.axis.plot(robot_world_pose['x'], robot_world_pose['z'], 'ro', ms=10, 
                      mec='darkred', mew=1)
        
        # Orientation arrow
        arrow_length = GRID_CELL_SIZE * 0.7
        dx = arrow_length * math.cos(robot_world_pose['theta'])
        dz = arrow_length * math.sin(robot_world_pose['theta'])
        arrow = plt.matplotlib.patches.FancyArrowPatch(
            (robot_world_pose['x'], robot_world_pose['z']), 
            (robot_world_pose['x'] + dx, robot_world_pose['z'] + dz),
            arrowstyle='->', mutation_scale=15, color='darkred', lw=2
        )
        self.axis.add_patch(arrow)
        
        # Highlight current grid cell
        if robot_grid_position:
            self._highlight_current_cell(robot_grid_position, line_sensors)
    
    def _highlight_current_cell(self, grid_position, line_sensors):
        """Highlight robot's current grid cell."""
        center_x, center_z = CoordinateConverter.grid_to_world_center(grid_position[0], grid_position[1])
        sensors_active = any(line_sensors)
        
        highlight_color = 'green' if sensors_active else 'yellow'
        highlight_alpha = 0.5 if sensors_active else 0.3
        
        highlight_rect = plt.Rectangle(
            (center_x - GRID_CELL_SIZE/2, center_z - GRID_CELL_SIZE/2),
            GRID_CELL_SIZE, GRID_CELL_SIZE,
            edgecolor=highlight_color, facecolor=highlight_color, 
            alpha=highlight_alpha, linewidth=3
        )
        self.axis.add_patch(highlight_rect)
        
        # Status text
        sensor_status = "ON LINE" if sensors_active else "NO LINE"
        status_color = 'green' if sensors_active else 'orange'
        
        self.axis.text(center_x, center_z + GRID_CELL_SIZE * 0.6, sensor_status, 
                      ha='center', va='bottom', fontsize=7, color=status_color, weight='bold',
                      bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))
    
    def _draw_goal_position(self):
        """Draw goal position marker."""
        goal_x, goal_z = CoordinateConverter.grid_to_world_center(GOAL_ROW, GOAL_COL)
        self.axis.plot(goal_x, goal_z, 'g*', ms=15, mec='darkgreen', mew=1.5)
    
    def _display_system_info(self, grid_position, line_sensors):
        """Display system information panel."""
        line_status = "ON BLACK LINE" if any(line_sensors) else "NO LINE DETECTED"
        info_background = 'lightgreen' if any(line_sensors) else 'lightcoral'
        
        info_text = (f"Grid Position: {grid_position} -> Goal: ({GOAL_ROW},{GOAL_COL})\n"
                    f"Sensor Status: {line_status}\n"
                    f"Obstacles Detected: {len(self.obstacle_detector.detected_obstacles)}")
        
        self.axis.text(0.02, 0.98, info_text, transform=self.axis.transAxes, va='top', fontsize=8,
                      bbox=dict(boxstyle='round,pad=0.4', facecolor=info_background, alpha=0.8))


class TurnController:
    """Handle robot turning operations with line detection."""
    
    def __init__(self):
        self.current_phase = 'NONE'
        self.active_command = None
        self.phase_start_time = 0.0
    
    def initiate_turn(self, turn_direction, current_time):
        """Start a new turning operation."""
        if self.active_command != turn_direction or self.current_phase == 'NONE':
            self.active_command = turn_direction
            self.current_phase = 'INITIATE_SPIN'
            self.phase_start_time = current_time
    
    def execute_turn(self, turn_direction, line_sensors, current_time):
        """Execute turn operation and return motor speeds."""
        if self.current_phase == 'INITIATE_SPIN':
            return self._execute_initial_spin(current_time)
        elif self.current_phase == 'SEARCHING_LINE':
            return self._execute_line_search(line_sensors, current_time)
        elif self.current_phase == 'ADJUSTING_ON_LINE':
            return self._execute_line_adjustment(line_sensors, current_time)
        
        return 0.0, 0.0
    
    def _execute_initial_spin(self, current_time):
        """Execute initial spin phase to get off current line."""
        spin_speeds = self._calculate_turn_speeds(0.8, 1.1)
        
        if current_time - self.phase_start_time > MIN_INITIAL_SPIN_DURATION:
            self.current_phase = 'SEARCHING_LINE'
            self.phase_start_time = current_time
        
        return spin_speeds
    
    def _execute_line_search(self, line_sensors, current_time):
        """Search for line during turn operation."""
        search_speeds = self._calculate_turn_speeds(0.5, 0.9)
        
        if any(line_sensors):
            self.current_phase = 'ADJUSTING_ON_LINE'
            self.phase_start_time = current_time
        elif (not TURN_UNTIL_LINE_FOUND and 
              current_time - self.phase_start_time > MAX_SEARCH_SPIN_DURATION):
            self.current_phase = 'NONE'
            return 0.0, 0.0
        
        return search_speeds
    
    def _execute_line_adjustment(self, line_sensors, current_time):
        """Fine-tune position on detected line."""
        left_sensor, center_sensor, right_sensor = line_sensors
        base_speed = TURN_ADJUST_BASE_SPEED
        moderate_diff = MODERATE_CORRECTION_DIFFERENTIAL * (base_speed / FORWARD_SPEED)
        aggressive_diff = AGGRESSIVE_CORRECTION_DIFFERENTIAL * (base_speed / FORWARD_SPEED)
        
        if not left_sensor and center_sensor and not right_sensor:
            # Perfect center - turn complete
            self.current_phase = 'NONE'
            self.active_command = None
            return base_speed * 0.3, base_speed * 0.3
        elif left_sensor and center_sensor and not right_sensor:
            return base_speed - moderate_diff, base_speed
        elif not left_sensor and center_sensor and right_sensor:
            return base_speed, base_speed - moderate_diff
        elif left_sensor and not center_sensor and not right_sensor:
            return base_speed - aggressive_diff, base_speed
        elif not left_sensor and not center_sensor and right_sensor:
            return base_speed, base_speed - aggressive_diff
        elif not any(line_sensors):
            # Line lost - return to search
            self.current_phase = 'SEARCHING_LINE'
            self.phase_start_time = current_time
            return self._calculate_turn_speeds(0.5, 0.9)
        else:
            return base_speed * 0.7, base_speed * 0.7
        
        # Timeout check
        if current_time - self.phase_start_time > MAX_ADJUST_DURATION:
            self.current_phase = 'NONE'
            self.active_command = None
            return 0.0, 0.0
    
    def _calculate_turn_speeds(self, inner_factor, outer_factor):
        """Calculate motor speeds for turning."""
        inner_speed = -FORWARD_SPEED * TURN_SPEED_FACTOR * inner_factor
        outer_speed = FORWARD_SPEED * TURN_SPEED_FACTOR * outer_factor
        
        if self.active_command == 'turn_left':
            return inner_speed, outer_speed
        else:  # turn_right
            return outer_speed, inner_speed
    
    def is_turning(self):
        """Check if currently executing a turn."""
        return self.current_phase != 'NONE'
    
    def reset(self):
        """Reset turn controller state."""
        self.current_phase = 'NONE'
        self.active_command = None


class NetworkManager:
    """Handle network communication with ESP32."""
    
    def __init__(self, esp32_ip, esp32_port):
        self.esp32_ip = esp32_ip
        self.esp32_port = esp32_port
        self.client_socket = None
        self.is_connected = False
    
    def establish_connection(self):
        """Establish connection to ESP32."""
        try:
            if self.client_socket:
                self.client_socket.close()
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(2.0)
            self.client_socket.connect((self.esp32_ip, self.esp32_port))
            self.client_socket.settimeout(0.05)
            self.is_connected = True
            print("ESP32 Connection Established")
            return True
        except Exception:
            self.is_connected = False
            self.client_socket = None
            return False
    
    def send_data(self, data):
        """Send data to ESP32."""
        try:
            message = json.dumps(data) + '\n'
            self.client_socket.sendall(message.encode('utf-8'))
            return True
        except Exception:
            self.is_connected = False
            return False
    
    def receive_data(self):
        """Receive data from ESP32."""
        try:
            response = self.client_socket.recv(1024)
            if response:
                messages = response.decode('utf-8').strip().split('\n')
                return [msg for msg in messages if msg.strip()]
            return []
        except socket.timeout:
            return []
        except Exception:
            self.is_connected = False
            return []
    
    def close_connection(self):
        """Close network connection."""
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        self.client_socket = None
        self.is_connected = False


def initialize_robot_systems():
    """Initialize all robot hardware systems."""
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # Initialize motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    for motor in [left_motor, right_motor]:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)
    
    # Initialize encoders
    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    for encoder in [left_encoder, right_encoder]:
        encoder.enable(timestep)
    
    # Initialize ground sensors
    ground_sensors = []
    for name in ['gs0', 'gs1', 'gs2']:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        ground_sensors.append(sensor)
    
    # Initialize distance sensors
    distance_sensors = []
    sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    for name in sensor_names:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        distance_sensors.append(sensor)
    
    # Select specific sensors for obstacle detection
    obstacle_sensors = [distance_sensors[0], distance_sensors[7], distance_sensors[5]]
    
    return {
        'robot': robot,
        'timestep': timestep,
        'left_motor': left_motor,
        'right_motor': right_motor,
        'left_encoder': left_encoder,
        'right_encoder': right_encoder,
        'ground_sensors': ground_sensors,
        'obstacle_sensors': obstacle_sensors
    }


def update_robot_odometry(world_pose, encoders, first_update):
    """Update robot position using wheel encoder odometry."""
    if first_update:
        prev_left = encoders['left_encoder'].getValue()
        prev_right = encoders['right_encoder'].getValue()
        return world_pose, prev_left, prev_right, False
    
    current_left = encoders['left_encoder'].getValue()
    current_right = encoders['right_encoder'].getValue()
    
    left_diff = current_left - encoders['prev_left']
    right_diff = current_right - encoders['prev_right']
    
    # Calculate movement
    distance = (left_diff * WHEEL_RADIUS + right_diff * WHEEL_RADIUS) / 2.0
    rotation = (right_diff * WHEEL_RADIUS - left_diff * WHEEL_RADIUS) / AXLE_LENGTH
    
    # Update position
    world_pose['x'] += distance * math.cos(world_pose['theta'] + rotation / 2.0)
    world_pose['z'] += distance * math.sin(world_pose['theta'] + rotation / 2.0)
    world_pose['theta'] = math.atan2(
        math.sin(world_pose['theta'] + rotation), 
        math.cos(world_pose['theta'] + rotation)
    )
    
    return world_pose, current_left, current_right, first_update


def calculate_line_following_speeds(line_sensors):
    """Calculate motor speeds for line following behavior."""
    left_sensor, center_sensor, right_sensor = line_sensors
    base_speed = FORWARD_SPEED
    
    if not left_sensor and center_sensor and not right_sensor:
        return base_speed, base_speed
    elif left_sensor and center_sensor and not right_sensor:
        return base_speed - MODERATE_CORRECTION_DIFFERENTIAL, base_speed
    elif not left_sensor and center_sensor and right_sensor:
        return base_speed, base_speed - MODERATE_CORRECTION_DIFFERENTIAL
    elif left_sensor and not center_sensor and not right_sensor:
        return base_speed - AGGRESSIVE_CORRECTION_DIFFERENTIAL, base_speed
    elif not left_sensor and not center_sensor and right_sensor:
        return base_speed, base_speed - AGGRESSIVE_CORRECTION_DIFFERENTIAL
    elif left_sensor and center_sensor and right_sensor:
        return base_speed * 0.7, base_speed * 0.7
    elif not any(line_sensors):
        return base_speed * 0.2, base_speed * 0.2
    else:
        return base_speed * 0.3, base_speed * 0.3


def main():
    """Main program execution loop."""
    # Initialize systems
    hardware = initialize_robot_systems()
    visualization = VisualizationManager()
    turn_controller = TurnController()
    network_manager = NetworkManager(ESP32_IP_ADDRESS, ESP32_PORT)
    
    # Initialize robot state
    robot_world_pose = {
        'x': CoordinateConverter.grid_to_world_center(INITIAL_GRID_ROW, INITIAL_GRID_COL)[0],
        'z': CoordinateConverter.grid_to_world_center(INITIAL_GRID_ROW, INITIAL_GRID_COL)[1],
        'theta': math.pi / 2.0
    }
    
    current_grid_position = CoordinateConverter.world_to_grid(robot_world_pose['x'], robot_world_pose['z'])
    planned_path = []
    esp32_command = 'stop'
    
    # Control loop variables
    iteration_count = 0
    last_connection_attempt = 0
    last_data_transmission = 0
    last_obstacle_check = 0
    first_odometry_update = True
    encoder_values = {'prev_left': 0.0, 'prev_right': 0.0}
    
    print("Hardware-in-the-Loop Navigation System Started")
    
    # Main control loop
    while hardware['robot'].step(hardware['timestep']) != -1:
        current_time = hardware['robot'].getTime()
        iteration_count += 1
        
        # Read sensor data
        line_sensor_values = [s.getValue() for s in hardware['ground_sensors']]
        line_detected = [1 if v < LINE_THRESHOLD else 0 for v in line_sensor_values]
        
        obstacle_sensor_values = [s.getValue() for s in hardware['obstacle_sensors']]
        
        # Update robot position
        robot_world_pose, encoder_values['prev_left'], encoder_values['prev_right'], first_odometry_update = \
            update_robot_odometry(robot_world_pose, {
                'left_encoder': hardware['left_encoder'],
                'right_encoder': hardware['right_encoder'],
                'prev_left': encoder_values['prev_left'],
                'prev_right': encoder_values['prev_right']
            }, first_odometry_update)
        
        current_grid_position = CoordinateConverter.world_to_grid(robot_world_pose['x'], robot_world_pose['z'])
        
        # Process obstacle detection
        if current_time - last_obstacle_check > 0.2:
            new_obstacles = visualization.obstacle_detector.process_sensor_readings(
                robot_world_pose, robot_world_pose['theta'], obstacle_sensor_values)
            if new_obstacles:
                visualization.obstacle_detector.add_recent_obstacles(new_obstacles)
            last_obstacle_check = current_time
        
        # Handle network communication
        if not network_manager.is_connected:
            if current_time - last_connection_attempt > 3.0:
                network_manager.establish_connection()
                last_connection_attempt = current_time
            hardware['left_motor'].setVelocity(0.0)
            hardware['right_motor'].setVelocity(0.0)
            if iteration_count % 10 == 0:
                visualization.update_display(robot_world_pose, current_grid_position, line_detected, planned_path)
            continue
        
        # Send data to ESP32
        if current_time - last_data_transmission > 0.1:
            navigation_data = {
                'type': 'webots_status',
                'robot_grid_pos': list(current_grid_position),
                'goal_grid_pos': [GOAL_ROW, GOAL_COL],
                'world_pose': {
                    'x': round(robot_world_pose['x'], 3),
                    'z': round(robot_world_pose['z'], 3),
                    'theta_rad': round(robot_world_pose['theta'], 3)
                },
                'sensors_binary': line_detected,
                'detected_obstacles': visualization.obstacle_detector.get_recent_obstacles()
            }
            
            if not network_manager.send_data(navigation_data):
                continue
            last_data_transmission = current_time
        
        # Receive commands from ESP32
        received_messages = network_manager.receive_data()
        for message in received_messages:
            try:
                esp_data = json.loads(message)
                if esp_data.get('type') == 'esp32_command':
                    new_command = esp_data.get('action', 'stop')
                    if (new_command != esp32_command and 
                        esp32_command in ['turn_left', 'turn_right'] and 
                        new_command not in ['turn_left', 'turn_right']):
                        turn_controller.reset()
                    esp32_command = new_command
                    planned_path = esp_data.get('path', planned_path)
            except json.JSONDecodeError:
                pass
        
        # Determine effective command based on sensor state
        sensors_detect_line = any(line_detected)
        
        if turn_controller.is_turning():
            if esp32_command == 'stop':
                turn_controller.reset()
                effective_command = 'stop'
            else:
                effective_command = turn_controller.active_command
        elif sensors_detect_line and esp32_command not in ['turn_left', 'turn_right', 'stop']:
            effective_command = 'forward'
        elif not sensors_detect_line and esp32_command == 'forward':
            effective_command = 'turn_left'
        else:
            effective_command = esp32_command
        
        # Execute movement commands
        left_speed, right_speed = 0.0, 0.0
        
        if effective_command == 'stop':
            turn_controller.reset()
        elif effective_command == 'forward':
            turn_controller.reset()
            left_speed, right_speed = calculate_line_following_speeds(line_detected)
        elif effective_command in ['turn_left', 'turn_right']:
            turn_controller.initiate_turn(effective_command, current_time)
            left_speed, right_speed = turn_controller.execute_turn(effective_command, line_detected, current_time)
        
        # Apply motor velocities
        hardware['left_motor'].setVelocity(left_speed)
        hardware['right_motor'].setVelocity(right_speed)
        
        # Update visualization
        if iteration_count % 3 == 0:
            visualization.update_display(robot_world_pose, current_grid_position, line_detected, planned_path)
        
        # Status reporting
        if iteration_count % 100 == 0:
            connection_status = "Connected" if network_manager.is_connected else "Disconnected"
            sensor_status = "Line Detected" if any(line_detected) else "No Line"
            print(f"Status: ESP32 {connection_status} | {sensor_status} | Grid: {current_grid_position}")
    
    # Cleanup
    network_manager.close_connection()
    if visualization.figure:
        plt.ioff()
        plt.show(block=True)


if __name__ == "__main__":
    main()