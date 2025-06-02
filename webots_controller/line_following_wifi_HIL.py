"""
Webots HIL Controller with ESP32 Integration
Dijkstra Path Planning with Sensor-Based Visualization
"""
from controller import Robot, DistanceSensor, Motor
import socket
import time
import math
import matplotlib.pyplot as plt
import json

# Network Configuration
ESP32_IP_ADDRESS = "192.168.53.193"
ESP32_PORT = 8080

# Robot Parameters
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.0590

# Grid Configuration (Now using X,Y instead of X,Z)
GRID_ROWS = 15
GRID_COLS = 19
GRID_CELL_SIZE = 0.057  # You also need to adjust this to match the cells size it shoud be around from 0.05 to 0.06

# Grid origin coordinates you can check this in webots
GRID_ORIGIN_X = 0.0
GRID_ORIGIN_Z = 00.0

GOAL_ROW = 14
GOAL_COL = 0

# Enhanced Parameters
FORWARD_SPEED = 1.3  # Slightly reduced for smoother operation
LINE_THRESHOLD = 600

# Distance Sensor Parameters
DISTANCE_SENSOR_THRESHOLD = 150  # Raw threshold value for obstacle detection (higher value = closer)
# Note: For testing, you can lower this value (e.g., 300-400) to make obstacle detection more sensitive
# Higher values = need to be closer to detect. Typical IR sensor range: 20-1000+ raw units
OBSTACLE_DETECTION_ENABLED = True
OBSTACLE_CELL_AHEAD = 1  # How many cells ahead to mark as obstacle

# Testing/Debug Parameters
OBSTACLE_TEST_MODE = False  # Set to True to inject test obstacles for debugging
TEST_OBSTACLE_INTERVAL = 5.0  # Inject test obstacle every N seconds
last_test_obstacle = 0

# Improved Distance Sensor Parameters
DISTANCE_SENSOR_THRESHOLD = 400  # More sensitive for better detection
OBSTACLE_DETECTION_ENABLED = True
OBSTACLE_CELL_AHEAD = 1  # Detect closer obstacles for better response
OBSTACLE_CONFIRMATION_FRAMES = 3  # Require multiple detections to confirm obstacle

# Smoother Turning Parameters
TURN_SPEED_FACTOR = 0.8
MIN_INITIAL_SPIN_DURATION = 1.8
MAX_SEARCH_SPIN_DURATION = 3.5
MAX_ADJUST_DURATION = 4.0
TURN_ADJUST_BASE_SPEED = FORWARD_SPEED * 0.7

# Enhanced Line Following Parameters
AGGRESSIVE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 1.8
MODERATE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 1.5
GENTLE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 1.2

# World grid definition (0 = Black Line, 1 = White Space)
world_grid = [
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


# Global variables
plt.ion()
fig = None
ax = None
robot_trail_world = []
planned_path_grid = []
webots_internal_turn_phase = 'NONE'
webots_turn_command_active = None
turn_phase_start_time = 0.0
obstacle_tracker = ObstacleTracker()

def world_to_grid(world_x, world_y):
    """Convert world coordinates to grid coordinates"""
    col = round((world_x - GRID_ORIGIN_X) / GRID_CELL_SIZE)
    row = round((world_y - GRID_ORIGIN_Y) / GRID_CELL_SIZE)
    col = max(0, min(col, GRID_COLS - 1))
    row = max(0, min(row, GRID_ROWS - 1))
    return row, col

def grid_to_world_center(row, col):
    """Convert grid coordinates to world coordinates (center of cell)"""
    world_x = GRID_ORIGIN_X + col * GRID_CELL_SIZE
    world_y = GRID_ORIGIN_Y + row * GRID_CELL_SIZE
    return world_x, world_y

def get_line_centered_position(robot_world_pos, current_grid_pos, line_detected):
    """Center robot position on grid cell when sensors detect line"""
    if any(line_detected):
        current_row, current_col = current_grid_pos
        
        if 0 <= current_row < GRID_ROWS and 0 <= current_col < GRID_COLS:
            if world_grid[current_row][current_col] == 0:
                # On black line, center on it
                grid_center_x, grid_center_y = grid_to_world_center(current_row, current_col)
                return grid_center_x, grid_center_y
            else:
                # Find nearest black cell
                min_dist = float('inf')
                best_x, best_y = robot_world_pos['x'], robot_world_pos['y']
                
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        r, c = current_row + dr, current_col + dc
                        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
                            if world_grid[r][c] == 0:
                                cx, cy = grid_to_world_center(r, c)
                                dist = (cx - robot_world_pos['x'])**2 + (cy - robot_world_pos['y'])**2
                                if dist < min_dist:
                                    min_dist = dist
                                    best_x, best_y = cx, cy
                
                return best_x, best_y
    
    return robot_world_pos['x'], robot_world_pos['y']

def detect_obstacles_smooth(robot_world_pos, robot_theta, distance_values):
    """Enhanced obstacle detection with smoothing and confirmation"""
    if not OBSTACLE_DETECTION_ENABLED:
        return []
    
    new_obstacles = []
    
    # Sensor configuration (front, front-left, front-right)
    sensor_configs = [
        {'angle': 0, 'name': 'front', 'sensor': 'ps5'},
        {'angle': math.pi/4, 'name': 'front-left', 'sensor': 'ps7'},
        {'angle': -math.pi/4, 'name': 'front-right', 'sensor': 'ps0'}
    ]
    
    for i, (distance_value, config) in enumerate(zip(distance_values, sensor_configs)):
        obstacle_detected = distance_value > DISTANCE_SENSOR_THRESHOLD
        
        if obstacle_detected:
            # Calculate obstacle position
            sensor_angle = robot_theta + config['angle']
            obstacle_distance = GRID_CELL_SIZE * OBSTACLE_CELL_AHEAD
            obstacle_x = robot_world_pos['x'] + obstacle_distance * math.cos(sensor_angle)
            obstacle_y = robot_world_pos['y'] + obstacle_distance * math.sin(sensor_angle)
            
            obstacle_row, obstacle_col = world_to_grid(obstacle_x, obstacle_y)
            
            # Validate and confirm obstacle
            if (0 <= obstacle_row < GRID_ROWS and 0 <= obstacle_col < GRID_COLS):
                if world_grid[obstacle_row][obstacle_col] == 0:  # Only mark pathable cells
                    position = (obstacle_row, obstacle_col)
                    
                    # Use enhanced tracking with confirmation
                    if obstacle_tracker.update_detection(position, True):
                        new_obstacles.append(position)
                        print(f"🔍 {config['name']} sensor detected obstacle at grid {position}")
    
    return new_obstacles

def update_visualization(rwp, crgp, path_esp):
    global fig, ax, robot_trail_world, planned_path_grid
    
    if fig is None:
        fig, ax = plt.subplots(figsize=(12, 9))
        ax.set_aspect('equal')
        ax.set_title('gyattt', fontsize=14, fontweight='bold')
        ax.set_xlabel('World X (m)')
        ax.set_ylabel('World Z (m)')
        
        # Draw grid
        for r in range(GRID_ROWS + 1):
            z = GRID_ORIGIN_Z + r * GRID_CELL_SIZE
            ax.plot([GRID_ORIGIN_X, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE], 
                   [z, z], 'k-', alpha=0.2, lw=0.5)
        
        for c in range(GRID_COLS + 1):
            x = GRID_ORIGIN_X + c * GRID_CELL_SIZE
            ax.plot([x, x], 
                   [GRID_ORIGIN_Z, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE], 
                   'k-', alpha=0.2, lw=0.5)
        
        # Draw cells
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                cx, cz = grid_to_world_center(r, c)
                color = 'black' if world_grid[r][c] == 0 else 'lightgrey'
                alpha = 0.6 if color == 'black' else 0.3
                
                rect = plt.Rectangle(
                    (cx - GRID_CELL_SIZE/2, cz - GRID_CELL_SIZE/2),
                    GRID_CELL_SIZE, GRID_CELL_SIZE,
                    facecolor=color, alpha=alpha, edgecolor='gray', linewidth=0.5
                )
                ax.add_patch(rect)
                
                # Add coordinate labels
                if r % 3 == 0 and c % 3 == 0:
                    ax.text(cx, cz, f'({r},{c})', 
                           ha='center', va='center', fontsize=6, 
                           color='blue', alpha=0.5)
        
        # Set limits
        margin = GRID_CELL_SIZE * 2
        ax.set_xlim(GRID_ORIGIN_X - margin, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE + margin)
        ax.set_ylim(GRID_ORIGIN_Z - margin, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE + margin)
        
        # Legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(fc='black', alpha=0.7, label='Grid Map: Black Line'),
            Patch(fc='lightgrey', alpha=0.3, label='Grid Map: White Space'),
            Patch(fc='green', alpha=0.6, label='Sensor Detection'),
            plt.Line2D([0], [0], color='cyan', lw=2, label='Robot Trail'),
            plt.Line2D([0], [0], color='magenta', marker='o', ms=5, ls='--', lw=2, label='Planned Path'),
            plt.Line2D([0], [0], color='red', marker='o', ms=8, ls='', label='Robot'),
            plt.Line2D([0], [0], color='green', marker='*', ms=12, ls='', label='Goal')
        ]
        ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1))
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.01)

    # Clear dynamic elements
    num_static_lines = (GRID_ROWS + 1) + (GRID_COLS + 1)
    num_static_patches = GRID_ROWS * GRID_COLS
    num_static_texts = sum(1 for r in range(GRID_ROWS) for c in range(GRID_COLS) 
                          if r % 3 == 0 and c % 3 == 0)
    
    for element_list in [ax.lines[num_static_lines:], ax.patches[num_static_patches:], ax.texts[num_static_texts:]]:
        while element_list:
            element_list.pop(0).remove()
    
    # Get sensor data
    try:
        raw_values = [s.getValue() for s in ground_sensors]
        line_detected = [1 if v < LINE_THRESHOLD else 0 for v in raw_values]
    except:
        line_detected = [0, 0, 0]
        raw_values = [0, 0, 0]
    
    # Get display position (smoothed)
    display_x, display_y = get_line_centered_position(robot_world_pos, current_grid_pos, line_detected)
    
    # Update trail with smoothing
    robot_trail_world.append((display_x, display_y))
    if len(robot_trail_world) > 150:  # Shorter trail for cleaner look
        robot_trail_world.pop(0)
    
    # Draw components
    draw_grid_and_obstacles()
    draw_robot_trail()
    draw_planned_path(path_from_esp)
    draw_robot_and_status(display_x, display_y, robot_world_pos, current_grid_pos, line_detected, raw_values)
    draw_goal()
    draw_info_panel(robot_world_pos, display_x, display_y, current_grid_pos, line_detected, raw_values)
    
    plt.draw()
    plt.pause(0.001)

def setup_visualization():
    """Initialize the visualization"""
    global fig, ax
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_aspect('equal')
    ax.set_title('Enhanced HIL Navigation with D* Lite (X,Y Coordinates)', fontsize=16, fontweight='bold')
    ax.set_xlabel('World X (m)')
    ax.set_ylabel('World Y (m)')
    
    # Draw grid lines
    for r in range(GRID_ROWS + 1):
        y = GRID_ORIGIN_Y + r * GRID_CELL_SIZE
        ax.plot([GRID_ORIGIN_X, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE], 
               [y, y], 'k-', alpha=0.2, lw=0.5)
    
    for c in range(GRID_COLS + 1):
        x = GRID_ORIGIN_X + c * GRID_CELL_SIZE
        ax.plot([x, x], 
               [GRID_ORIGIN_Y, GRID_ORIGIN_Y + GRID_ROWS * GRID_CELL_SIZE], 
               'k-', alpha=0.2, lw=0.5)
    
    # Set limits
    margin = GRID_CELL_SIZE * 1.5
    ax.set_xlim(GRID_ORIGIN_X - margin, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE + margin)
    ax.set_ylim(GRID_ORIGIN_Y - margin, GRID_ORIGIN_Y + GRID_ROWS * GRID_CELL_SIZE + margin)
    
    # Enhanced legend
    create_legend()
    plt.tight_layout()
    plt.show(block=False)

def clear_dynamic_elements():
    """Clear dynamic visualization elements"""
    num_static_lines = (GRID_ROWS + 1) + (GRID_COLS + 1)
    
    # Clear patches and dynamic lines
    for patch in ax.patches[:]:
        patch.remove()
    
    texts_to_remove = [t for t in ax.texts if t.get_position()[0] > GRID_ORIGIN_X - GRID_CELL_SIZE]
    for text in texts_to_remove:
        text.remove()
    
    lines_to_remove = ax.lines[num_static_lines:]
    for line in lines_to_remove:
        line.remove()

def draw_grid_and_obstacles():
    """Draw grid cells and obstacles"""
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cx, cy = grid_to_world_center(r, c)
            
            # Determine cell color and style
            if (r, c) in obstacle_tracker.detected_obstacles:
                color, alpha = 'red', 0.8
            else:
                color = 'black' if world_grid[r][c] == 0 else 'lightgrey'
                alpha = 0.6 if color == 'black' else 0.2
            
            rect = plt.Rectangle(
                (cx - GRID_CELL_SIZE/2, cy - GRID_CELL_SIZE/2),
                GRID_CELL_SIZE, GRID_CELL_SIZE,
                facecolor=color, alpha=alpha, edgecolor='gray', linewidth=0.3
            )
            ax.add_patch(rect)
            
            # Add coordinate labels (sparse)
            if r % 4 == 0 and c % 4 == 0:
                ax.text(cx, cy, f'({r},{c})', 
                       ha='center', va='center', fontsize=6, 
                       color='blue', alpha=0.6)

def draw_robot_trail():
    """Draw robot movement trail"""
    if len(robot_trail_world) > 1:
        trail_x, trail_y = zip(*robot_trail_world)
        ax.plot(trail_x, trail_y, 'cyan', lw=2.5, alpha=0.7, label='Robot Trail')

def draw_planned_path(path_from_esp):
    """Draw the planned path"""
    global planned_path_grid
    if path_from_esp and len(path_from_esp) > 1:
        planned_path_grid = path_from_esp
        path_world = [grid_to_world_center(r, c) for r, c in planned_path_grid]
        if path_world:
            path_x, path_y = zip(*path_world)
            ax.plot(path_x, path_y, 'mo--', lw=2.5, ms=4, alpha=0.9, label='D* Lite Path')
            # Mark start and end
            ax.plot(path_x[0], path_y[0], 'm^', ms=10, label='Path Start')
            ax.plot(path_x[-1], path_y[-1], 'm*', ms=12, label='Path End')

def draw_robot_and_status(display_x, display_y, robot_world_pos, current_grid_pos, line_detected, raw_values):
    """Draw robot with enhanced status indicators"""
    # Draw robot
    ax.plot(display_x, display_y, 'ro', ms=12, mec='darkred', mew=2)
    
    # Draw orientation arrow
    arrow_length = GRID_CELL_SIZE * 0.8
    dx = arrow_length * math.cos(robot_world_pos['theta'])
    dy = arrow_length * math.sin(robot_world_pos['theta'])
    arrow = plt.matplotlib.patches.FancyArrowPatch(
        (display_x, display_y), (display_x + dx, display_y + dy),
        arrowstyle='->', mutation_scale=20, color='darkred', lw=3
    )
    ax.add_patch(arrow)
    
    # Highlight current cell with status
    if current_grid_pos:
        cx, cy = grid_to_world_center(current_grid_pos[0], current_grid_pos[1])
        sensors_on_line = any(line_detected)
        
        highlight_color = 'lime' if sensors_on_line else 'orange'
        highlight_alpha = 0.4 if sensors_on_line else 0.2
        
        highlight_rect = plt.Rectangle(
            (cx - GRID_CELL_SIZE/2, cy - GRID_CELL_SIZE/2),
            GRID_CELL_SIZE, GRID_CELL_SIZE,
            edgecolor=highlight_color, facecolor=highlight_color, 
            alpha=highlight_alpha, linewidth=4
        )
        ax.add_patch(highlight_rect)

def draw_goal():
    """Draw the goal position"""
    goal_x, goal_y = grid_to_world_center(GOAL_ROW, GOAL_COL)
    ax.plot(goal_x, goal_y, 'g*', ms=18, mec='darkgreen', mew=2)

def draw_info_panel(robot_world_pos, display_x, display_y, current_grid_pos, line_detected, raw_values):
    """Draw enhanced information panel"""
    line_status = "🟢 ON LINE" if any(line_detected) else "🔴 OFF LINE"
    
    # Show both actual and display positions for debugging
    actual_grid = world_to_grid(rwp['x'], rwp['z'])
    display_grid = world_to_grid(display_x, display_z)
    
    info_text = (f"Grid Position: {crgp} -> Goal: ({GOAL_ROW},{GOAL_COL})\n"
                f"Sensor Status: {line_status}\n"
                f"Actual World: X={rwp['x']:.3f}, Z={rwp['z']:.3f}\n"
                f"Display World: X={display_x:.3f}, Z={display_z:.3f}\n"
                f"Sensors (L,C,R): {line_detected} | Raw: {[f'{v:.0f}' for v in raw_values]}\n"
                f"Turn Phase: {webots_internal_turn_phase}")
    
    info_bg = 'lightgreen' if any(line_detected) else 'lightcoral'
    
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, va='top', fontsize=9,
           bbox=dict(boxstyle='round,pad=0.5', facecolor=info_bg, alpha=0.9))

def create_legend():
    """Create enhanced legend"""
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(fc='black', alpha=0.6, label='Black Line (Pathable)'),
        Patch(fc='lightgrey', alpha=0.2, label='White Space (Non-pathable)'),
        Patch(fc='red', alpha=0.8, label='Detected Obstacle'),
        Patch(fc='lime', alpha=0.4, label='Robot on Line'),
        plt.Line2D([0], [0], color='cyan', lw=2.5, label='Robot Trail'),
        plt.Line2D([0], [0], color='magenta', marker='o', ms=4, ls='--', lw=2.5, label='D* Lite Path'),
        plt.Line2D([0], [0], color='red', marker='o', ms=12, ls='', label='Robot'),
        plt.Line2D([0], [0], color='green', marker='*', ms=18, ls='', label='Goal')
    ]
    ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1))

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Robot state (now using X,Y)
robot_world_pos = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
prev_left_encoder = 0.0
prev_right_encoder = 0.0
first_odometry = True

# Motors and encoders
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')

for motor in [left_motor, right_motor]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

for encoder in [left_encoder, right_encoder]:
    encoder.enable(timestep)

# Ground sensors
ground_sensors = []
for name in ['gs0', 'gs1', 'gs2']:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    gs_wb.append(sensor)

# Network variables
client_socket = None
is_connected = False
esp32_command = 'stop'

def connect_to_esp32():
    """Enhanced ESP32 connection with better error handling"""
    global client_socket, is_connected
    print(f"🔄 Connecting to ESP32 at {ESP32_IP_ADDRESS}:{ESP32_PORT}")
    try:
        if client_socket:
            client_socket.close()
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(3.0)
        client_socket.connect((ESP32_IP_ADDRESS, ESP32_PORT))
        client_socket.settimeout(0.05)
        is_connected = True
        print("✅ ESP32 connected successfully")
        return True
    except Exception as e:
        print(f"❌ ESP32 connection failed: {e}")
        is_connected = False
        client_socket = None
        return False

# Set initial position (X,Y coordinates)
INITIAL_GRID_ROW = 3
INITIAL_GRID_COL = 18

robot_world_pos['x'], robot_world_pos['y'] = grid_to_world_center(INITIAL_GRID_ROW, INITIAL_GRID_COL)
robot_world_pos['theta'] = math.pi / 2.0  # Facing down

# Verify position
crgp = world_to_grid(rwp['x'], rwp['z'])
print(f"Robot initialized at grid {crgp}, world ({rwp['x']:.3f}, {rwp['z']:.3f})")
print(f"Target goal: ({GOAL_ROW}, {GOAL_COL})")

# Coordinate system verification
print("\nCoordinate System Verification:")
print(f"Grid origin: X={GRID_ORIGIN_X}, Z={GRID_ORIGIN_Z}")
print(f"Grid size: {GRID_ROWS}x{GRID_COLS}, Cell size: {GRID_CELL_SIZE}m")

test_positions = [(0,0), (0,16), (14,0), (7,8)]
for row, col in test_positions:
    wx, wz = grid_to_world_center(row, col)
    back_row, back_col = world_to_grid(wx, wz)
    print(f"  Grid ({row},{col}) -> World ({wx:.3f},{wz:.3f}) -> Grid ({back_row},{back_col})")

print("-" * 60)

# Main loop variables
iteration = 0
last_connection_attempt = 0
last_data_send = 0

# Main control loop
while robot.step(timestep) != -1:
    if iteration == 0:
        connect_to_esp32()
        update_visualization_smooth(robot_world_pos, current_grid_pos, planned_path_grid)
    
    iteration += 1
    current_time = robot.getTime()

    # Read sensors
    raw_values = [s.getValue() for s in ground_sensors]
    line_detected = [1 if v < LINE_THRESHOLD else 0 for v in raw_values]
    left_sensor, center_sensor, right_sensor = line_detected

    # Update odometry
    if not first_odometry:
        left_value = left_encoder.getValue()
        right_value = right_encoder.getValue()
        
        left_diff = left_value - prev_left_encoder
        right_diff = right_value - prev_right_encoder
        
        distance = (left_diff * WHEEL_RADIUS + right_diff * WHEEL_RADIUS) / 2.0
        rotation = (right_diff * WHEEL_RADIUS - left_diff * WHEEL_RADIUS) / AXLE_LENGTH
        
        robot_world_pos['x'] += distance * math.cos(robot_world_pos['theta'] + rotation / 2.0)
        robot_world_pos['y'] += distance * math.sin(robot_world_pos['theta'] + rotation / 2.0)
        robot_world_pos['theta'] = math.atan2(math.sin(robot_world_pos['theta'] + rotation), 
                                            math.cos(robot_world_pos['theta'] + rotation))
        
        prev_left_encoder = left_value
        prev_right_encoder = right_value
    else:
        prev_left_encoder = left_encoder.getValue()
        prev_right_encoder = right_encoder.getValue()
        first_odometry = False
    
    # Update grid position
    new_grid_pos = world_to_grid(robot_world_pos['x'], robot_world_pos['y'])
    if new_grid_pos != current_grid_pos:
        current_grid_pos = new_grid_pos

    # Enhanced obstacle detection
    if current_time - last_obstacle_check > 0.15:  # Slightly less frequent
        if OBSTACLE_DETECTION_ENABLED:
            new_obstacles = detect_obstacles_smooth(robot_world_pos, robot_world_pos['theta'], distance_values)
            last_obstacle_check = current_time

    # Detect obstacles periodically
    if current_time - last_obstacle_check > 0.2:  # Check every 200ms
        if OBSTACLE_DETECTION_ENABLED:
            new_obstacles = detect_obstacles_from_distance_sensors(rwp, rwp['theta'], distance_values)
            if new_obstacles:
                print(f"🚧 {len(new_obstacles)} new obstacles detected!")
                recent_new_obstacles.extend(new_obstacles)  # Add to persistent storage
        last_obstacle_check = current_time
    
    # Test mode: inject artificial obstacles for debugging
    if OBSTACLE_TEST_MODE and current_time - last_test_obstacle > TEST_OBSTACLE_INTERVAL:
        # Inject test obstacle ahead of robot
        test_obstacle_row = crgp[0] + 2  # 2 cells ahead
        test_obstacle_col = crgp[1]
        if (0 <= test_obstacle_row < GRID_ROWS and 0 <= test_obstacle_col < GRID_COLS and
            (test_obstacle_row, test_obstacle_col) not in detected_obstacles_grid):
            print(f"🧪 TEST MODE: Injecting artificial obstacle at ({test_obstacle_row}, {test_obstacle_col})")
            recent_new_obstacles.append((test_obstacle_row, test_obstacle_col))
            detected_obstacles_grid.add((test_obstacle_row, test_obstacle_col))
            last_test_obstacle = current_time

    # Handle ESP32 connection
    if not is_connected:
        if current_time - last_connection_attempt > 5.0:
            connect_to_esp32()
            last_connection_attempt = current_time
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if iteration % 15 == 0:
            update_visualization_smooth(robot_world_pos, current_grid_pos, planned_path_grid)
        continue

    # Enhanced motor control with smoother transitions
    left_speed, right_speed = 0.0, 0.0

    # Send data to ESP32
    if current_time - last_data_send > 0.08:  # Slightly faster updates
        try:
            recent_obstacles = obstacle_tracker.get_recent_obstacles()
            data = {
                'type': 'webots_status',
                'robot_grid_pos': list(current_grid_pos),
                'goal_grid_pos': [GOAL_ROW, GOAL_COL],
                'world_pose': {
                    'x': round(robot_world_pos['x'], 4),
                    'y': round(robot_world_pos['y'], 4),  # Changed from 'z' to 'y'
                    'theta_rad': round(robot_world_pos['theta'], 4)
                },
                'sensors_binary': line_detected
            }
            client_socket.sendall((json.dumps(data) + '\n').encode('utf-8'))
            last_data_send = current_time
        except Exception as e:
            print(f"Send error: {e}")
            is_connected = False
            if client_socket:
                client_socket.close()
                client_socket = None
            continue
    
    # Receive commands from ESP32
    try:
        response = client_socket.recv(1024)
        if response:
            for message in response.decode('utf-8').strip().split('\n'):
                if not message.strip():
                    continue
                try:
                    esp_data = json.loads(message)
                    if esp_data.get('type') == 'esp32_command':
                        new_command = esp_data.get('action', 'stop')
                        if (new_command != esp32_command and 
                            esp32_command in ['turn_left', 'turn_right'] and 
                            new_command not in ['turn_left', 'turn_right']):
                            webots_internal_turn_phase = 'NONE'
                            webots_turn_command_active = None
                        esp32_command = new_command
                        planned_path_grid = esp_data.get('path', planned_path_grid)
                except json.JSONDecodeError as e:
                    print(f"JSON error: {e}")
                except Exception as e:
                    print(f"Processing error: {e}")
    except socket.timeout:
        pass
    except Exception as e:
        print(f"Receive error: {e}")
        is_connected = False
        if client_socket:
            client_socket.close()

    # Enhanced motor control logic
    if esp32_command not in ['turn_left', 'turn_right'] and webots_internal_turn_phase != 'NONE':
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None

    if esp32_command == 'stop':
        left_speed, right_speed = 0.0, 0.0
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None
        
    elif esp32_command == 'forward':
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None
        base_speed = FORWARD_SPEED
        
        # Enhanced line following with smoother corrections
        if not left_sensor and center_sensor and not right_sensor:
            # Perfect center - full speed
            left_speed, right_speed = base_speed, base_speed
        elif left_sensor and center_sensor and not right_sensor:
            # Slight left deviation - gentle correction
            left_speed, right_speed = base_speed - GENTLE_CORRECTION_DIFFERENTIAL, base_speed
        elif not left_sensor and center_sensor and right_sensor:
            # Slight right deviation - gentle correction
            left_speed, right_speed = base_speed, base_speed - GENTLE_CORRECTION_DIFFERENTIAL
        elif left_sensor and not center_sensor and not right_sensor:
            # Strong left deviation - moderate correction
            left_speed, right_speed = base_speed - MODERATE_CORRECTION_DIFFERENTIAL, base_speed
        elif not left_sensor and not center_sensor and right_sensor:
            # Strong right deviation - moderate correction
            left_speed, right_speed = base_speed, base_speed - MODERATE_CORRECTION_DIFFERENTIAL
        elif left_sensor and center_sensor and right_sensor:
            # All sensors on line - intersection or wide line
            left_speed, right_speed = base_speed * 0.8, base_speed * 0.8
        elif not any(line_detected):
            # No line detected - slow search
            left_speed, right_speed = base_speed * 0.3, base_speed * 0.3
        else:
            # Other combinations - conservative approach
            left_speed, right_speed = base_speed * 0.5, base_speed * 0.5

    elif esp32_command in ['turn_left', 'turn_right']:
        # Enhanced turning logic with smoother phases
        if webots_turn_command_active != esp32_command or webots_internal_turn_phase == 'NONE':
            webots_turn_command_active = esp32_command
            webots_internal_turn_phase = 'INITIATE_SPIN'
            turn_phase_start_time = current_time
            print(f"🔄 Enhanced turn {esp32_command} initiated")

        if webots_internal_turn_phase == 'INITIATE_SPIN':
            # Initial spin with moderate speeds
            spin_inner = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.6
            spin_outer = FORWARD_SPEED * TURN_SPEED_FACTOR * 0.9
            left_speed, right_speed = (spin_inner, spin_outer) if webots_turn_command_active == 'turn_left' else (spin_outer, spin_inner)
            
            if current_time - turn_phase_start_time > MIN_INITIAL_SPIN_DURATION:
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_time
                
        elif webots_internal_turn_phase == 'SEARCHING_LINE':
            # Search for line with reduced speeds
            search_inner = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.3
            search_outer = FORWARD_SPEED * TURN_SPEED_FACTOR * 0.7
            left_speed, right_speed = (search_inner, search_outer) if webots_turn_command_active == 'turn_left' else (search_outer, search_inner)
            
            # Enhanced line acquisition logic
            line_acquired = (center_sensor or 
                           (webots_turn_command_active == 'turn_left' and left_sensor and not right_sensor) or 
                           (webots_turn_command_active == 'turn_right' and right_sensor and not left_sensor))
            
            if line_acquired:
                webots_internal_turn_phase = 'ADJUSTING_ON_LINE'
                turn_phase_start_time = current_time
                print(f"✅ Line acquired during {webots_turn_command_active}")
            elif current_time - turn_phase_start_time > MAX_SEARCH_SPIN_DURATION:
                print(f"⏰ Turn timeout - stopping")
                webots_internal_turn_phase = 'NONE'
                left_speed, right_speed = 0, 0
                
        elif webots_internal_turn_phase == 'ADJUSTING_ON_LINE':
            # Fine adjustment phase with smoother control
            base = TURN_ADJUST_BASE_SPEED
            gentle_diff = GENTLE_CORRECTION_DIFFERENTIAL * (base / FORWARD_SPEED)
            moderate_diff = MODERATE_CORRECTION_DIFFERENTIAL * (base / FORWARD_SPEED)
            
            if not left_sensor and center_sensor and not right_sensor:
                # Perfect alignment - slow forward
                left_speed, right_speed = base * 0.6, base * 0.6
            elif left_sensor and center_sensor and not right_sensor:
                # Minor left correction
                left_speed, right_speed = base - gentle_diff, base
            elif not left_sensor and center_sensor and right_sensor:
                # Minor right correction
                left_speed, right_speed = base, base - gentle_diff
            elif left_sensor and not center_sensor and not right_sensor:
                # Stronger left correction
                left_speed, right_speed = base - moderate_diff, base
            elif not left_sensor and not center_sensor and right_sensor:
                # Stronger right correction
                left_speed, right_speed = base, base - moderate_diff
            elif not any(line_detected):
                print(f"⚠️  Line lost during adjustment - searching again")
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_time
            else:
                # Default case
                left_speed, right_speed = base * 0.8, base * 0.8
                
            if current_time - turn_phase_start_time > MAX_ADJUST_DURATION:
                print(f"⏰ Adjustment timeout - completing turn")
                webots_internal_turn_phase = 'NONE'

    # Apply motor velocities with bounds checking
    left_speed = max(-5.0, min(5.0, left_speed))
    right_speed = max(-5.0, min(5.0, right_speed))
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Update visualization (less frequent for smoother performance)
    if iteration % 4 == 0:
        update_visualization_smooth(robot_world_pos, current_grid_pos, planned_path_grid)
    
    # Status logging
    if iteration % 25 == 0:
        connection_status = "Connected" if is_connected else "Disconnected"
        sensor_status = "ON LINE" if any(line_detected) else "NO LINE"
        print(f"Time: {current_time:.1f}s | ESP32: {connection_status} | Command: {esp32_command} | "
              f"Grid: {crgp} | {sensor_status} {line_detected} | Turn: {webots_internal_turn_phase}")

# Cleanup
if client_socket:
    try:
        client_socket.close()
        print("✅ ESP32 connection closed")
    except:
        pass

if obstacle_tracker.detected_obstacles:
    print(f"📊 Total obstacles detected: {len(obstacle_tracker.detected_obstacles)}")
    print(f"📍 Obstacle positions: {list(obstacle_tracker.detected_obstacles)}")
else:
    print("✅ No obstacles detected during simulation")

if fig:
    print("Simulation ended")
    plt.ioff()
    plt.show(block=True)

print("🎯 Enhanced Controller finished successfully!")
print("="*60)