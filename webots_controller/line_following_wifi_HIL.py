"""
Webots HIL Controller with ESP32 Integration
Dijkstra Path Planning with Sensor-Based Visualization
"""
# === WEBOTS HIL CONTROLLER WITH DYNAMIC D* REPLANNING ===
# This controller integrates with ESP32 for Hardware-in-the-Loop simulation
# Key features implemented:
# 1. Distance sensor monitoring (e-puck proximity sensor ps0)
# 2. Obstacle detection threshold: distance_value > 500 triggers D* replanning
# 3. Real-time communication with ESP32 D* Lite path planner
# 4. Visual feedback showing replanning status in simulation
# 5. Enhanced robot position tracking and line following
# 
# Distance sensor integration:
# - ps0 (front proximity sensor) used as main distance sensor
# - Values > 500 indicate obstacle presence
# - ESP32 receives sensor data and performs D* Lite replanning
# - Updated paths sent back to Webots for execution
# ============================================================

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

# this is for the turing parameters one point i was stuck beacuse it was not turning at all the issue was diffrent but it does't hurt to have this
TURN_SPEED_FACTOR = 0.9
MIN_INITIAL_SPIN_DURATION = 2.35
MAX_SEARCH_SPIN_DURATION = 4.5
MAX_ADJUST_DURATION = 5.0
TURN_ADJUST_BASE_SPEED = FORWARD_SPEED * 0.8

# Enhanced turning parameters for 180-degree turns during obstacle avoidance
OBSTACLE_TURN_SPEED_FACTOR = 1.2  # Higher speed for faster turns
OBSTACLE_MIN_INITIAL_SPIN_DURATION = 3.5  # Longer duration for 180-degree turns
OBSTACLE_MAX_SEARCH_SPIN_DURATION = 6.0  # Extended search time for line reacquisition

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

def update_visualization(rwp, crgp, path_esp, distance_value=0.0):
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
    
    # Check for D* replanning trigger
    replanning_status = ""
    if distance_value > 500:
        replanning_status = "🔄 D* REPLANNING TRIGGERED!"
    
    # Show both actual and display positions for debugging
    actual_grid = world_to_grid(rwp['x'], rwp['z'])
    display_grid = world_to_grid(display_x, display_z)
    
    info_text = (f"Grid Position: {crgp} -> Goal: ({GOAL_ROW},{GOAL_COL})\n"
                f"Sensor Status: {line_status}\n"
                f"Distance Sensor: {distance_value:.3f}m {replanning_status}\n"
                f"Actual World: X={rwp['x']:.3f}, Z={rwp['z']:.3f}\n"
                f"Display World: X={display_x:.3f}, Z={display_z:.3f}\n"
                f"Sensors (L,C,R): {line_detected} | Raw: {[f'{v:.0f}' for v in raw_values]}\n"
                f"Turn Phase: {webots_internal_turn_phase}")
    
    info_bg = 'lightgreen' if any(line_detected) else 'lightcoral'
    if distance_value > 500:
        info_bg = 'orange'  # Highlight when replanning is triggered
    
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

# Proximity sensors (e-puck distance sensors)
proximity_sensors = []
for i in range(8):  # e-puck has 8 proximity sensors ps0-ps7
    ps = robot.getDevice(f'ps{i}')
    ps.enable(timestep)
    proximity_sensors.append(ps)

# Use front proximity sensor as main distance sensor
distance_sensor = proximity_sensors[0]  # ps0 is front-facing

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
        print("ESP32 connected successfully")
        return True
    except Exception as e:
        print(f"ESP32 connection failed: {e}")
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
        update_visualization(rwp, crgp, planned_path_grid, 0.0)
    
    iteration += 1
    current_time = robot.getTime()

    # Read sensors first (needed for position correction)
    raw_values = [s.getValue() for s in gs_wb]
    line_detected = [1 if v < LINE_THRESHOLD else 0 for v in raw_values]
    left_sensor, center_sensor, right_sensor = line_detected
    
    # Read distance sensor
    distance_value = distance_sensor.getValue()
    
    # Check for obstacle detection threshold (trigger D* replanning on ESP32)
    if distance_value > 500:
        print(f"  OBSTACLE DETECTED! Distance Sensor: {distance_value:.3f} - D* replanning will be triggered")

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

    # Handle ESP32 connection
    if not is_connected:
        if current_time - last_connection_attempt > 3.0:
            connect_to_esp32()
            last_connection_attempt = current_time
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if iteration % 10 == 0:
            update_visualization(rwp, crgp, planned_path_grid, 0.0)
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
                'distance_sensor': round(distance_value, 3)
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
                        
                        # Display ESP32 response with distance sensor info and replanning status
                        esp32_distance = esp_data.get('distance_sensor_reading', 'N/A')
                        algorithm = esp_data.get('algorithm', 'Unknown')
                        path_length = len(esp_data.get('path', []))
                        replanning_indicator = "🔄 REPLANNING" if esp32_distance != 'N/A' and esp32_distance > 500 else ""
                        print(f"ESP32 Response: Action={new_command}, Algorithm={algorithm}, "
                              f"Distance Reading={esp32_distance}, Path Length={path_length} {replanning_indicator}")
                        
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
        # Enhanced turn logic with obstacle avoidance detection
        if webots_turn_command_active != esp32_command or webots_internal_turn_phase == 'NONE':
            webots_turn_command_active = esp32_command
            webots_internal_turn_phase = 'INITIATE_SPIN'
            turn_phase_start_time = current_time
            
            # Detect if this might be a 180-degree turn based on recent obstacle detection
            is_major_turn = distance_value > 500 or esp32_distance == 'N/A' or (esp32_distance != 'N/A' and esp32_distance > 500)
            
            if is_major_turn:
                print(f"⚠️  Major turn {esp32_command} initiated (180° obstacle avoidance)")
                webots_turn_command_active += "_major"  # Flag for major turn
            else:
                print(f"Turn {esp32_command} initiated (normal)")

        # Determine if this is a major turn
        is_major_turn = "_major" in webots_turn_command_active
        
        # Select appropriate parameters
        if is_major_turn:
            turn_speed_factor = OBSTACLE_TURN_SPEED_FACTOR
            min_spin_duration = OBSTACLE_MIN_INITIAL_SPIN_DURATION
            max_search_duration = OBSTACLE_MAX_SEARCH_SPIN_DURATION
            base_command = webots_turn_command_active.replace("_major", "")
        else:
            turn_speed_factor = TURN_SPEED_FACTOR
            min_spin_duration = MIN_INITIAL_SPIN_DURATION
            max_search_duration = MAX_SEARCH_SPIN_DURATION
            base_command = webots_turn_command_active

        if webots_internal_turn_phase == 'INITIATE_SPIN':
            spin_in = -FORWARD_SPEED * turn_speed_factor * 0.7
            spin_out = FORWARD_SPEED * turn_speed_factor * 1.0
            left_speed, right_speed = (spin_in, spin_out) if base_command == 'turn_left' else (spin_out, spin_in)
            
            if current_time - turn_phase_start_time > min_spin_duration:
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_time
                
        elif webots_internal_turn_phase == 'SEARCHING_LINE':
            search_in = -FORWARD_SPEED * turn_speed_factor * 0.4
            search_out = FORWARD_SPEED * turn_speed_factor * 0.8
            left_speed, right_speed = (search_in, search_out) if base_command == 'turn_left' else (search_out, search_in)
            
            line_acquired = (center_sensor or 
                           (base_command == 'turn_left' and left_sensor and not right_sensor) or 
                           (base_command == 'turn_right' and right_sensor and not left_sensor))
            
            if line_acquired:
                webots_internal_turn_phase = 'ADJUSTING_ON_LINE'
                turn_phase_start_time = current_time
                print(f"Line acquired during {base_command}")
            elif current_time - turn_phase_start_time > max_search_duration:
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
        update_visualization(rwp, crgp, planned_path_grid, distance_value)
    
    # Status logging
    if iteration % 25 == 0:
        connection_status = "Connected" if is_connected else "Disconnected"
        sensor_status = "ON LINE" if any(line_detected) else "NO LINE"
        print(f"Time: {current_time:.1f}s | ESP32: {connection_status} | Command: {esp32_command} | "
              f"Grid: {crgp} | {sensor_status} {line_detected} | Turn: {webots_internal_turn_phase} | "
              f"Distance: {distance_value:.3f}m")

# Cleanup
if client_socket:
    try:
        client_socket.close()
    except:
        pass

if fig:
    print("Simulation ended")
    plt.ioff()
    plt.show(block=True)

print("Controller finished")