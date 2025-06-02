"""
Webots HIL Controller with ESP32 Integration
Dijkstra Path Planning with Sensor-Based Visualization and Obstacle Detection
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
AXLE_LENGTH = 0.05900  # somtimes if u change this for some fucking reason u see good signs even though the correct is around 0.0520 or somthing

# Grid Configuration
GRID_ROWS = 15
GRID_COLS = 19
GRID_CELL_SIZE = 0.057  # You also need to adjust this to match the cells size it shoud be around from 0.05 to 0.06

# Grid origin coordinates you can check this in webots
GRID_ORIGIN_X = 0.0
GRID_ORIGIN_Z = 00.0

GOAL_ROW = 14
GOAL_COL = 0

# Parameters
FORWARD_SPEED = 1.2 # don't make it too fast eitherway it will not work properly
LINE_THRESHOLD = 600

# Distance Sensor Parameters
DISTANCE_SENSOR_THRESHOLD = 500  # Raw threshold value for obstacle detection (higher value = closer)
# Note: For testing, you can lower this value (e.g., 300-400) to make obstacle detection more sensitive
# Higher values = need to be closer to detect. Typical IR sensor range: 20-1000+ raw units
OBSTACLE_DETECTION_ENABLED = True
OBSTACLE_CELL_AHEAD = 2  # How many cells ahead to mark as obstacle

# Testing/Debug Parameters
OBSTACLE_TEST_MODE = False  # Set to True to inject test obstacles for debugging
TEST_OBSTACLE_INTERVAL = 5.0  # Inject test obstacle every N seconds
last_test_obstacle = 0

# this is for the turing parameters one point i was stuck beacuse it was not turning at all the issue was diffrent but it does't hurt to have this
TURN_SPEED_FACTOR = 0.9
MIN_INITIAL_SPIN_DURATION = 2.35
MAX_SEARCH_SPIN_DURATION = 4.5
MAX_ADJUST_DURATION = 5.0
TURN_ADJUST_BASE_SPEED = FORWARD_SPEED * 0.8

# Line Centering Parameters to keep the bot alwyas on the middle of the line
AGGRESSIVE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 2.3
MODERATE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 2.2

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

# Track detected obstacles
detected_obstacles_grid = set()  # Set of (row, col) tuples
recent_new_obstacles = []  # Store recently detected obstacles for ESP32 communication

# Global variables
plt.ion()
fig = None
ax = None
robot_trail_world = []
planned_path_grid = []
webots_internal_turn_phase = 'NONE'
webots_turn_command_active = None
turn_phase_start_time = 0.0

def world_to_grid(world_x, world_z):
    """Convert world coordinates to grid coordinates"""
    col = round((world_x - GRID_ORIGIN_X) / GRID_CELL_SIZE)
    row = round((world_z - GRID_ORIGIN_Z) / GRID_CELL_SIZE)
    col = max(0, min(col, GRID_COLS - 1))
    row = max(0, min(row, GRID_ROWS - 1))
    return row, col

def grid_to_world_center(row, col):
    """Convert grid coordinates to world coordinates (center of cell)"""
    world_x = GRID_ORIGIN_X + col * GRID_CELL_SIZE
    world_z = GRID_ORIGIN_Z + row * GRID_CELL_SIZE
    return world_x, world_z

def get_line_centered_position(rwp, crgp, ldf):
    """Center robot position on grid cell when sensors detect line"""
    # If sensors detect a line, trust it completely :) 
    if any(ldf):
        # Find the nearest black line cell to current position
        current_row, current_col = crgp
        
        # Check if current cell is black line
        if 0 <= current_row < GRID_ROWS and 0 <= current_col < GRID_COLS:
            if world_grid[current_row][current_col] == 0:
                # Already on black line, center on it
                grid_center_x, grid_center_z = grid_to_world_center(current_row, current_col)
                return grid_center_x, grid_center_z
            else:
                # this is not working now but the plan is when the Sensors detect line but grid shows white - find nearest black cell 
                min_dist = float('inf')
                best_x, best_z = rwp['x'], rwp['z']
                
                # Search adjacent cells
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        r, c = current_row + dr, current_col + dc
                        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
                            if world_grid[r][c] == 0:  # Black line cell
                                cx, cz = grid_to_world_center(r, c)
                                dist = (cx - rwp['x'])**2 + (cz - rwp['z'])**2
                                if dist < min_dist:
                                    min_dist = dist
                                    best_x, best_z = cx, cz
                
                return best_x, best_z
    
    # No line detected, use actual position
    return rwp['x'], rwp['z']

def detect_obstacles_from_distance_sensors(rwp, robot_theta, distance_values):
    """
    Detect obstacles based on distance sensor readings.
    Returns list of grid positions that contain obstacles.
    """
    if not OBSTACLE_DETECTION_ENABLED:
        return []
    
    new_obstacles = []
    
    # Distance sensor configuration
    # ps5 = front, ps7 = front-left, ps0 = front-right
    sensor_config = [
        {'angle': 0, 'name': 'front'},           # ps5
        {'angle': math.pi/4, 'name': 'front-left'},    # ps7  
        {'angle': -math.pi/4, 'name': 'front-right'}   # ps0
    ]
    
    for i, (distance_value, config) in enumerate(zip(distance_values, sensor_config)):
        sensor_name = ['ps5', 'ps7', 'ps0'][i]
        if distance_value > DISTANCE_SENSOR_THRESHOLD:  # Note: higher value = closer for IR sensors
            print(f"🔍 {config['name']} sensor ({sensor_name}) detected obstacle: {distance_value:.0f} > {DISTANCE_SENSOR_THRESHOLD}")
            # Calculate obstacle position based on sensor angle and robot orientation
            sensor_angle = robot_theta + config['angle']
            
            # Calculate obstacle position (multiple cells ahead based on OBSTACLE_CELL_AHEAD)
            for cell_distance in range(1, OBSTACLE_CELL_AHEAD + 1):
                obstacle_distance = GRID_CELL_SIZE * cell_distance
                obstacle_x = rwp['x'] + obstacle_distance * math.cos(sensor_angle)
                obstacle_z = rwp['z'] + obstacle_distance * math.sin(sensor_angle)
                
                obstacle_row, obstacle_col = world_to_grid(obstacle_x, obstacle_z)
                
                # Debug the coordinate calculation
                if distance_value > DISTANCE_SENSOR_THRESHOLD:
                    print(f"    🎯 Cell {cell_distance}: World({obstacle_x:.3f}, {obstacle_z:.3f}) -> Grid({obstacle_row}, {obstacle_col})")
                
                # Check if this is a valid obstacle position
                if (0 <= obstacle_row < GRID_ROWS and 0 <= obstacle_col < GRID_COLS):
                    # Only mark as obstacle if it's supposed to be a pathable cell
                    if world_grid[obstacle_row][obstacle_col] == 0:
                        if (obstacle_row, obstacle_col) not in detected_obstacles_grid:
                            new_obstacles.append((obstacle_row, obstacle_col))
                            detected_obstacles_grid.add((obstacle_row, obstacle_col))
                            print(f"🚨 OBSTACLE detected by {config['name']} sensor at grid ({obstacle_row}, {obstacle_col})")
                        else:
                            print(f"    ℹ️  Already detected obstacle at grid ({obstacle_row}, {obstacle_col})")
                    else:
                        print(f"    ⚪ Skipping non-pathable cell at grid ({obstacle_row}, {obstacle_col}) - grid value: {world_grid[obstacle_row][obstacle_col]}")
                else:
                    print(f"    ❌ Out of bounds: grid ({obstacle_row}, {obstacle_col}) not in range [0-{GRID_ROWS-1}, 0-{GRID_COLS-1}]")
    
    return new_obstacles

def update_visualization(rwp, crgp, path_esp):
    global fig, ax, robot_trail_world, planned_path_grid
    
    if fig is None:
        fig, ax = plt.subplots(figsize=(12, 9))
        ax.set_aspect('equal')
        ax.set_title('HIL Navigation with Obstacle Detection', fontsize=14, fontweight='bold')
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
        
        # Set limits
        margin = GRID_CELL_SIZE * 2
        ax.set_xlim(GRID_ORIGIN_X - margin, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE + margin)
        ax.set_ylim(GRID_ORIGIN_Z - margin, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE + margin)
        
        # Legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(fc='black', alpha=0.7, label='Grid Map: Black Line'),
            Patch(fc='lightgrey', alpha=0.3, label='Grid Map: White Space'),
            Patch(fc='red', alpha=0.7, label='Detected Obstacle'),
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
    
    # Clear all patches and redraw
    for patch in ax.patches[:]:
        patch.remove()
        
    # Clear texts
    texts_to_remove = [t for t in ax.texts if t.get_position()[0] > GRID_ORIGIN_X - GRID_CELL_SIZE]
    for text in texts_to_remove:
        text.remove()
    
    # Clear dynamic lines
    lines_to_remove = ax.lines[num_static_lines:]
    for line in lines_to_remove:
        line.remove()
    
    # Draw cells
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cx, cz = grid_to_world_center(r, c)
            
            # Check if this cell is a detected obstacle
            if (r, c) in detected_obstacles_grid:
                color = 'red'
                alpha = 0.7
            else:
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
    
    # Get sensor data
    try:
        raw_values = [s.getValue() for s in gs_wb]
        line_detected = [1 if v < LINE_THRESHOLD else 0 for v in raw_values]
    except:
        line_detected = [0, 0, 0]
        raw_values = [0, 0, 0]
    
    # Get display position
    display_x, display_z = get_line_centered_position(rwp, crgp, line_detected)
    
    # Update trail
    robot_trail_world.append((display_x, display_z))
    if len(robot_trail_world) > 200:
        robot_trail_world.pop(0)
    
    if len(robot_trail_world) > 1:
        trail_x, trail_z = zip(*robot_trail_world)
        ax.plot(trail_x, trail_z, 'cyan', lw=2, alpha=0.7)
    
    # Draw planned path
    if path_esp and len(path_esp) > 1:
        planned_path_grid = path_esp
        path_world = [grid_to_world_center(r, c) for r, c in planned_path_grid]
        if path_world:
            path_x, path_z = zip(*path_world)
            ax.plot(path_x, path_z, 'mo--', lw=2, ms=5, alpha=0.8)
            ax.plot(path_x[0], path_z[0], 'm^', ms=8)
            ax.plot(path_x[-1], path_z[-1], 'm*', ms=8)
    
    # Draw robot
    ax.plot(display_x, display_z, 'ro', ms=10, mec='darkred', mew=1)
    
    # Draw orientation arrow
    arrow_length = GRID_CELL_SIZE * 0.7
    dx = arrow_length * math.cos(rwp['theta'])
    dz = arrow_length * math.sin(rwp['theta'])
    arrow = plt.matplotlib.patches.FancyArrowPatch(
        (display_x, display_z), (display_x + dx, display_z + dz),
        arrowstyle='->', mutation_scale=15, color='darkred', lw=2
    )
    ax.add_patch(arrow)
    
    # Highlight current cell
    if crgp:
        cx, cz = grid_to_world_center(crgp[0], crgp[1])
        sensors_on_line = any(line_detected)
        
        highlight_color = 'green' if sensors_on_line else 'yellow'
        highlight_alpha = 0.5 if sensors_on_line else 0.3
        
        highlight_rect = plt.Rectangle(
            (cx - GRID_CELL_SIZE/2, cz - GRID_CELL_SIZE/2),
            GRID_CELL_SIZE, GRID_CELL_SIZE,
            edgecolor=highlight_color, facecolor=highlight_color, 
            alpha=highlight_alpha, linewidth=3
        )
        ax.add_patch(highlight_rect)
        
        # Cell status
        grid_val = world_grid[crgp[0]][crgp[1]] if (0 <= crgp[0] < GRID_ROWS and 0 <= crgp[1] < GRID_COLS) else -1
        sensor_status = "ON LINE" if sensors_on_line else "NO LINE"
        grid_status = "black" if grid_val == 0 else "white" if grid_val == 1 else "OOB"
        
        # Show mismatch warning
        mismatch = (sensors_on_line and grid_val == 1) or (not sensors_on_line and grid_val == 0)
        status_text = f"Sensors: {sensor_status}\nGrid: {grid_status}"
        if mismatch:
            status_text += "\nMISMATCH!"
        
        status_color = 'red' if mismatch else ('green' if sensors_on_line else 'orange')
        
        ax.text(cx, cz + GRID_CELL_SIZE * 0.6, status_text, 
               ha='center', va='bottom', fontsize=7, color=status_color, weight='bold',
               bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

    # Draw goal
    goal_x, goal_z = grid_to_world_center(GOAL_ROW, GOAL_COL)
    ax.plot(goal_x, goal_z, 'g*', ms=15, mec='darkgreen', mew=1.5)
    
    # Info panel
    line_status = "ON BLACK LINE" if any(line_detected) else "NO LINE DETECTED"
    
    # Show both actual and display positions for debugging
    actual_grid = world_to_grid(rwp['x'], rwp['z'])
    display_grid = world_to_grid(display_x, display_z)
    
    info_text = (f"Grid Position: {crgp} -> Goal: ({GOAL_ROW},{GOAL_COL})\n"
                f"Sensor Status: {line_status}\n"
                f"Actual World: X={rwp['x']:.3f}, Z={rwp['z']:.3f}\n"
                f"Display World: X={display_x:.3f}, Z={display_z:.3f}\n"
                f"Sensors (L,C,R): {line_detected} | Raw: {[f'{v:.0f}' for v in raw_values]}\n"
                f"Obstacles: {len(detected_obstacles_grid)}\n"
                f"Turn Phase: {webots_internal_turn_phase}")
    
    info_bg = 'lightgreen' if any(line_detected) else 'lightcoral'
    
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, va='top', fontsize=8,
           bbox=dict(boxstyle='round,pad=0.4', facecolor=info_bg, alpha=0.8))
    
    plt.draw()
    plt.pause(0.001)

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Robot state
rwp = {'x': 0.0, 'z': 0.0, 'theta': 0.0}
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
gs_wb = []
for name in ['gs0', 'gs1', 'gs2']:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    gs_wb.append(sensor)

# initialize distance sensors
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

print("\n--- Initializing Distance Sensors ---")
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
    print(f"✓ Distance sensor {psNames[i]} enabled")

# We'll use ps5 (front), ps7 (front-left), ps0 (front-right) for obstacle detection
distance_sensors = [ps[5], ps[7], ps[0]]  # front, front-left, front-right
print(f"✓ Using sensors ps5 (front), ps7 (front-left), ps0 (front-right) for obstacle detection")
print(f"✓ Distance threshold: {DISTANCE_SENSOR_THRESHOLD} (raw value)")
print("-" * 40)

# Network variables
client_socket = None
is_connected = False
esp32_command = 'stop'

def connect_to_esp32():
    global client_socket, is_connected
    print(f"Attempting connection to ESP32 at {ESP32_IP_ADDRESS}:{ESP32_PORT}")
    try:
        if client_socket:
            client_socket.close()
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(2.0)
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

# CONFIGURABLE STARTING POSITION change this if u want a diffrent starting point.. u can read the nodes from the dashboard
INITIAL_GRID_ROW = 3
INITIAL_GRID_COL = 18

# Set initial position from grid coordinates
rwp['x'], rwp['z'] = grid_to_world_center(INITIAL_GRID_ROW, INITIAL_GRID_COL)
rwp['theta'] = math.pi / 2.0  # Facing down

# Verify position
crgp = world_to_grid(rwp['x'], rwp['z'])
print(f"Robot initialized at grid {crgp}, world ({rwp['x']:.3f}, {rwp['z']:.3f})")
print(f"Target goal: ({GOAL_ROW}, {GOAL_COL})")
print(f"Obstacle detection: {'ENABLED' if OBSTACLE_DETECTION_ENABLED else 'DISABLED'}")
if OBSTACLE_TEST_MODE:
    print(f"🧪 TEST MODE: Will inject artificial obstacles every {TEST_OBSTACLE_INTERVAL}s")
print(f"Distance sensor threshold: {DISTANCE_SENSOR_THRESHOLD} (lower = more sensitive)")

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
last_obstacle_check = 0
distance_display_counter = 0
last_test_obstacle = 0  # For test mode obstacle injection

# Main control loop
while robot.step(timestep) != -1:
    if iteration == 0:
        connect_to_esp32()
        update_visualization(rwp, crgp, planned_path_grid)
    
    iteration += 1
    current_time = robot.getTime()

    # Read sensors first (needed for position correction)
    raw_values = [s.getValue() for s in gs_wb]
    line_detected = [1 if v < LINE_THRESHOLD else 0 for v in raw_values]
    left_sensor, center_sensor, right_sensor = line_detected
    
    # read distance sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    
    # Get values for obstacle detection sensors
    distance_values = [psValues[5], psValues[7], psValues[0]]  # front, front-left, front-right
    
    # Display distance sensor values every 10 iterations
    if OBSTACLE_DETECTION_ENABLED:
        distance_display_counter += 1
        if distance_display_counter >= 10:
            distance_display_counter = 0
            # Show detailed sensor readings
            print(f"📏 Distance Sensors - Front: {distance_values[0]:.0f}, "
                  f"Front-Left: {distance_values[1]:.0f}, "
                  f"Front-Right: {distance_values[2]:.0f} (threshold: {DISTANCE_SENSOR_THRESHOLD})")
            
            # Show which sensors are detecting obstacles
            detection_status = []
            front_obstacle = distance_values[0] > DISTANCE_SENSOR_THRESHOLD
            left_obstacle = distance_values[1] > DISTANCE_SENSOR_THRESHOLD
            right_obstacle = distance_values[2] > DISTANCE_SENSOR_THRESHOLD
            
            if front_obstacle:
                detection_status.append(f"FRONT({distance_values[0]:.0f})")
            if left_obstacle:
                detection_status.append(f"LEFT({distance_values[1]:.0f})")
            if right_obstacle:
                detection_status.append(f"RIGHT({distance_values[2]:.0f})")
            
            if detection_status:
                print(f"⚠️  Obstacle detection: {' + '.join(detection_status)}")
            else:
                print(f"✅ No obstacles detected")
            
            # Show recent obstacles count
            if recent_new_obstacles:
                print(f"🔄 Pending obstacles to send: {len(recent_new_obstacles)}")
            
            # Old individual alerts (keep for compatibility)
            if front_obstacle:
                print(f"⚠️  Front sensor detecting obstacle! Raw value: {distance_values[0]:.0f}")
            if left_obstacle:
                print(f"⚠️  Front-Left sensor detecting obstacle! Raw value: {distance_values[1]:.0f}")
            if right_obstacle:
                print(f"⚠️  Front-Right sensor detecting obstacle! Raw value: {distance_values[2]:.0f}")

    # Update odometry
    if not first_odometry:
        left_value = left_encoder.getValue()
        right_value = right_encoder.getValue()
        
        left_diff = left_value - prev_left_encoder
        right_diff = right_value - prev_right_encoder
        
        distance = (left_diff * WHEEL_RADIUS + right_diff * WHEEL_RADIUS) / 2.0
        rotation = (right_diff * WHEEL_RADIUS - left_diff * WHEEL_RADIUS) / AXLE_LENGTH
        
        rwp['x'] += distance * math.cos(rwp['theta'] + rotation / 2.0)
        rwp['z'] += distance * math.sin(rwp['theta'] + rotation / 2.0)
        rwp['theta'] = math.atan2(math.sin(rwp['theta'] + rotation), 
                                  math.cos(rwp['theta'] + rotation))
        
        prev_left_encoder = left_value
        prev_right_encoder = right_value
    else:
        prev_left_encoder = left_encoder.getValue()
        prev_right_encoder = right_encoder.getValue()
        first_odometry = False
    
    # Update grid position
    new_grid_pos = world_to_grid(rwp['x'], rwp['z'])
    if new_grid_pos != crgp:
        crgp = new_grid_pos
        # Check for position mismatch
        if 0 <= crgp[0] < GRID_ROWS and 0 <= crgp[1] < GRID_COLS:
            grid_value = world_grid[crgp[0]][crgp[1]]
            if (grid_value == 0 and not any(line_detected)) or (grid_value == 1 and any(line_detected)):
                print(f"Position mismatch at grid {crgp}: Grid expects {'BLACK' if grid_value == 0 else 'WHITE'}, "
                      f"sensors detect {'LINE' if any(line_detected) else 'NO LINE'}")

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
        if current_time - last_connection_attempt > 3.0:
            connect_to_esp32()
            last_connection_attempt = current_time
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if iteration % 10 == 0:
            update_visualization(rwp, crgp, planned_path_grid)
        continue

    # Motor control
    left_speed, right_speed = 0.0, 0.0

    # Send data to ESP32
    if current_time - last_data_send > 0.1:
        try:
            data = {
                'type': 'webots_status',
                'robot_grid_pos': list(crgp),
                'goal_grid_pos': [GOAL_ROW, GOAL_COL],
                'world_pose': {
                    'x': round(rwp['x'], 3),
                    'z': round(rwp['z'], 3),
                    'theta_rad': round(rwp['theta'], 3)
                },
                'sensors_binary': line_detected,
                'detected_obstacles': recent_new_obstacles.copy()  # Send copy of recent obstacles
            }
            client_socket.sendall((json.dumps(data) + '\n').encode('utf-8'))
            last_data_send = current_time
            
            # Clear recent obstacles after sending to avoid sending duplicates
            if recent_new_obstacles:
                print(f"📤 Sent {len(recent_new_obstacles)} obstacles to ESP32")
                recent_new_obstacles.clear()
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
                        
                        # Show algorithm being used
                        algorithm = esp_data.get('algorithm', 'Unknown')
                        if iteration % 100 == 0:
                            print(f"📍 Using {algorithm} algorithm for path planning")
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
    # Motor control
    left_speed, right_speed = 0.0, 0.0

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
        
        # Line following logic
        if not left_sensor and center_sensor and not right_sensor:
            left_speed, right_speed = base_speed, base_speed
        elif left_sensor and center_sensor and not right_sensor:
            left_speed, right_speed = base_speed - MODERATE_CORRECTION_DIFFERENTIAL, base_speed
        elif not left_sensor and center_sensor and right_sensor:
            left_speed, right_speed = base_speed, base_speed - MODERATE_CORRECTION_DIFFERENTIAL
        elif left_sensor and not center_sensor and not right_sensor:
            left_speed, right_speed = base_speed - AGGRESSIVE_CORRECTION_DIFFERENTIAL, base_speed
        elif not left_sensor and not center_sensor and right_sensor:
            left_speed, right_speed = base_speed, base_speed - AGGRESSIVE_CORRECTION_DIFFERENTIAL
        elif left_sensor and center_sensor and right_sensor:
            left_speed, right_speed = base_speed * 0.7, base_speed * 0.7
        elif not any(line_detected):
            left_speed, right_speed = base_speed * 0.2, base_speed * 0.2
        else:
            left_speed, right_speed = base_speed * 0.3, base_speed * 0.3

    elif esp32_command in ['turn_left', 'turn_right']:
        # Turn logic
        if webots_turn_command_active != esp32_command or webots_internal_turn_phase == 'NONE':
            webots_turn_command_active = esp32_command
            webots_internal_turn_phase = 'INITIATE_SPIN'
            turn_phase_start_time = current_time
            print(f"Turn {esp32_command} initiated")

        if webots_internal_turn_phase == 'INITIATE_SPIN':
            spin_in = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.7
            spin_out = FORWARD_SPEED * TURN_SPEED_FACTOR * 1.0
            left_speed, right_speed = (spin_in, spin_out) if webots_turn_command_active == 'turn_left' else (spin_out, spin_in)
            
            if current_time - turn_phase_start_time > MIN_INITIAL_SPIN_DURATION:
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_time
                
        elif webots_internal_turn_phase == 'SEARCHING_LINE':
            search_in = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.4
            search_out = FORWARD_SPEED * TURN_SPEED_FACTOR * 0.8
            left_speed, right_speed = (search_in, search_out) if webots_turn_command_active == 'turn_left' else (search_out, search_in)
            
            line_acquired = (center_sensor or 
                           (webots_turn_command_active == 'turn_left' and left_sensor and not right_sensor) or 
                           (webots_turn_command_active == 'turn_right' and right_sensor and not left_sensor))
            
            if line_acquired:
                webots_internal_turn_phase = 'ADJUSTING_ON_LINE'
                turn_phase_start_time = current_time
                print(f"Line acquired during {webots_turn_command_active}")
            elif current_time - turn_phase_start_time > MAX_SEARCH_SPIN_DURATION:
                print(f"Turn timeout - stopping")
                webots_internal_turn_phase = 'NONE'
                left_speed, right_speed = 0, 0
                
        elif webots_internal_turn_phase == 'ADJUSTING_ON_LINE':
            base = TURN_ADJUST_BASE_SPEED
            mod_diff = MODERATE_CORRECTION_DIFFERENTIAL * (base / FORWARD_SPEED)
            agg_diff = AGGRESSIVE_CORRECTION_DIFFERENTIAL * (base / FORWARD_SPEED)
            
            if not left_sensor and center_sensor and not right_sensor:
                left_speed, right_speed = base * 0.5, base * 0.5
            elif left_sensor and center_sensor and not right_sensor:
                left_speed, right_speed = base - mod_diff, base
            elif not left_sensor and center_sensor and right_sensor:
                left_speed, right_speed = base, base - mod_diff
            elif left_sensor and not center_sensor and not right_sensor:
                left_speed, right_speed = base - agg_diff, base
            elif not left_sensor and not center_sensor and right_sensor:
                left_speed, right_speed = base, base - agg_diff
            elif not any(line_detected):
                print(f"Line lost during adjustment - searching again")
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_time
            else:
                left_speed, right_speed = base * 0.7, base * 0.7
                
            if current_time - turn_phase_start_time > MAX_ADJUST_DURATION:
                print(f"Adjustment timeout - stopping")
                webots_internal_turn_phase = 'NONE'
                left_speed, right_speed = 0, 0

    # Apply motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Update visualization
    if iteration % 3 == 0:
        update_visualization(rwp, crgp, planned_path_grid)
    
    # Status logging
    if iteration % 25 == 0:
        connection_status = "Connected" if is_connected else "Disconnected"
        sensor_status = "ON LINE" if any(line_detected) else "NO LINE"
        obstacles_str = f"Obstacles: {len(detected_obstacles_grid)}" if detected_obstacles_grid else "No obstacles"
        print(f"Time: {current_time:.1f}s | ESP32: {connection_status} | Command: {esp32_command} | "
              f"Grid: {crgp} | {sensor_status} {line_detected} | {obstacles_str}")

# Cleanup
if client_socket:
    try:
        client_socket.close()
    except:
        pass

if fig:
    print("\nSimulation ended")
    print(f"Total obstacles detected: {len(detected_obstacles_grid)}")
    if detected_obstacles_grid:
        print("Obstacle positions:", list(detected_obstacles_grid))
    plt.ioff()
    plt.show(block=True)

print("Controller finished")