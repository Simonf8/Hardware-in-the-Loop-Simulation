"""
Clean Webots HIL Controller with Live Dashboard, ESP32 Dijkstra Path Planning,
Enhanced Turns, Aggressive Line Centering, and SENSOR-BASED VISUALIZATION.
Focus: Trust line sensors as ground truth, not world_grid array for visualization.
"""
from controller import Robot
import socket
import time
import math
import matplotlib.pyplot as plt
import json

# --- Network Configuration ---
ESP32_IP_ADDRESS = "192.168.53.193"  # VERIFY THIS IS YOUR ESP32's CURRENT IP
ESP32_PORT = 8080

# --- Robot Parameters ---
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.0810

# --- Grid Configuration (Must match ESP32) ---
GRID_ROWS = 15
GRID_COLS = 17
GRID_CELL_SIZE = 0.05

# CRITICAL FIX: These origins should match where your actual arena is positioned in Webots
# You need to measure these values from your Webots world file
# These should be the world coordinates of the CENTER of grid cell (0,0)
GRID_ORIGIN_X = 10  # ADJUST THIS - measure from your Webots world
GRID_ORIGIN_Z = 10 # ADJUST THIS - measure from your Webots world

# Alternative: If you know the world coordinates of a specific grid cell, 
# calculate the origin from there
# For example, if you know grid cell (7,8) is at world coordinates (0.1, 0.2):
# GRID_ORIGIN_X = 0.1 - (8 + 0.5) * GRID_CELL_SIZE
# GRID_ORIGIN_Z = 0.2 - (7 + 0.5) * GRID_CELL_SIZE

GOAL_ROW = 14
GOAL_COL = 0

# --- Control Parameters ---
FORWARD_SPEED = 2.8
LINE_THRESHOLD = 600

# --- Enhanced Turn Parameters ---
TURN_SPEED_FACTOR = 0.8
MIN_INITIAL_SPIN_DURATION = 0.35
MAX_SEARCH_SPIN_DURATION = 2.5
MAX_ADJUST_DURATION = 2.0
TURN_ADJUST_BASE_SPEED = FORWARD_SPEED * 0.8

# --- Aggressive Line Centering Parameters ---
AGGRESSIVE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 2.3
MODERATE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 2.2

# --- world_grid definition ---
# NOTE: This is used for path planning, but we'll trust sensors for visualization
# 0 = Black Line (Pathable), 1 = White Space (Obstacle)
world_grid = [
    [1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0],  # Row 0
    [1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0],  # Row 1
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 2
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0],  # Row 3
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0],  # Row 4
    [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0],  # Row 5
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0],  # Row 6
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 7
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0],  # Row 8
    [0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0],  # Row 9
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0],  # Row 10
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0],  # Row 11
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 12
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1],  # Row 13
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1]   # Row 14
]

plt.ion()
fig = None
ax = None
robot_trail_world = []
planned_path_grid = []
webots_internal_turn_phase = 'NONE'
webots_turn_command_active = None
turn_phase_start_time = 0.0

def world_to_grid(world_x, world_z):
    """Convert world coordinates to grid coordinates with improved precision"""
    # Calculate relative position from grid origin
    rel_x = world_x - GRID_ORIGIN_X
    rel_z = world_z - GRID_ORIGIN_Z
    
    # Convert to grid coordinates
    col = rel_x / GRID_CELL_SIZE
    row = rel_z / GRID_CELL_SIZE
    
    # Round to nearest grid cell for more accurate positioning
    col = max(0, min(int(round(col)), GRID_COLS - 1))
    row = max(0, min(int(round(row)), GRID_ROWS - 1))
    
    return row, col

def grid_to_world_center(row, col):
    """Convert grid coordinates to world coordinates (center of cell)"""
    world_x = GRID_ORIGIN_X + col * GRID_CELL_SIZE
    world_z = GRID_ORIGIN_Z + row * GRID_CELL_SIZE
    return world_x, world_z

def get_robot_position_from_webots(robot):
    """Get the actual robot position from Webots supervisor (if available)"""
    # This is for future improvement - requires supervisor node
    # For now, we use the odometry calculation
    pass

def get_line_centered_position(rwp, crgp, ldf):
    """
    TRUST SENSORS: Always show robot centered on black line when sensors detect line.
    Completely ignore the world_grid array - sensors are ground truth!
    """
    sensors_on_line = any(ldf)  # Check if any sensor detects line
    
    if not sensors_on_line:  # If no sensors detect line, use actual position
        return rwp['x'], rwp['z']
    
    # SENSORS SAY WE'RE ON LINE: Center on current grid cell
    # This is what matters, not what world_grid says!
    
    # Get center of current grid cell
    grid_center_x, grid_center_z = grid_to_world_center(crgp[0], crgp[1])
    
    # Very strong centering when sensors detect line
    blend_factor = 0.95  # Almost complete centering for clean visualization
    
    display_x = rwp['x'] * (1 - blend_factor) + grid_center_x * blend_factor
    display_z = rwp['z'] * (1 - blend_factor) + grid_center_z * blend_factor
    
    return display_x, display_z

def update_visualization(rwp, crgp, path_esp):
    global fig, ax, robot_trail_world, planned_path_grid
    if fig is None:
        fig, ax = plt.subplots(figsize=(14,10))
        ax.set_aspect('equal')
        ax.set_title('🤖 HIL Robot Dijkstra Path Planning 🎯 (Sensor-Based View)', fontsize=16, fontweight='bold')
        ax.set_xlabel('World X (m)')
        ax.set_ylabel('World Z (m)')
        
        # Draw grid lines
        for r_idx in range(GRID_ROWS+1):
            z_line = GRID_ORIGIN_Z + r_idx * GRID_CELL_SIZE
            ax.plot([GRID_ORIGIN_X, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE], 
                   [z_line, z_line], 'k-', alpha=0.2, lw=0.5)
        
        for c_idx in range(GRID_COLS+1):
            x_line = GRID_ORIGIN_X + c_idx * GRID_CELL_SIZE
            ax.plot([x_line, x_line], 
                   [GRID_ORIGIN_Z, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE], 
                   'k-', alpha=0.2, lw=0.5)
        
        # Draw grid cells based on world_grid (for reference)
        for r_idx_draw in range(GRID_ROWS):
            for c_idx_draw in range(GRID_COLS):
                wcx, wcz = grid_to_world_center(r_idx_draw, c_idx_draw)
                cell_val = world_grid[r_idx_draw][c_idx_draw]
                
                # Color: black for pathable (0), light grey for obstacles (1)
                clr = 'black' if cell_val == 0 else 'lightgrey'
                alpha = 0.7 if clr == 'black' else 0.3
                
                # Draw cell as rectangle centered on grid point
                rect = plt.Rectangle(
                    (wcx - GRID_CELL_SIZE/2, wcz - GRID_CELL_SIZE/2),
                    GRID_CELL_SIZE, GRID_CELL_SIZE,
                    facecolor=clr, alpha=alpha, edgecolor='gray', linewidth=0.5
                )
                ax.add_patch(rect)
                
                # Add grid coordinate labels for reference points
                if r_idx_draw % 3 == 0 and c_idx_draw % 3 == 0:
                    ax.text(wcx, wcz, f'({r_idx_draw},{c_idx_draw})', 
                           ha='center', va='center', fontsize=7, 
                           color='blue', alpha=0.6)
        
        # Set axis limits with margin
        mgn = GRID_CELL_SIZE * 1.5
        ax.set_xlim(GRID_ORIGIN_X - mgn, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE + mgn)
        ax.set_ylim(GRID_ORIGIN_Z - mgn, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE + mgn)
        
        # Create legend
        from matplotlib.patches import Patch
        lgd = [
            Patch(fc='black', alpha=0.7, label='Grid Map: Black Line'),
            Patch(fc='lightgrey', alpha=0.3, label='Grid Map: White Space'),
            Patch(fc='green', alpha=0.7, label='SENSOR DETECTED LINE'),
            plt.Line2D([0], [0], color='cyan', lw=2, label='Robot Trail'),
            plt.Line2D([0], [0], color='magenta', marker='o', ms=5, ls='--', lw=2, label='Dijkstra Path'),
            plt.Line2D([0], [0], color='red', marker='o', ms=8, ls='', label='Robot'),
            plt.Line2D([0], [0], color='green', marker='*', ms=12, ls='', label='Goal')
        ]
        ax.legend(handles=lgd, loc='upper left', bbox_to_anchor=(1.02, 1))
        plt.tight_layout(rect=[0, 0, 0.85, 1])
        plt.show(block=False)
        plt.pause(0.01)

    # Clear dynamic elements
    num_static_lines = (GRID_ROWS + 1) + (GRID_COLS + 1)
    num_static_patches = GRID_ROWS * GRID_COLS
    num_static_texts = sum(1 for r_idx in range(GRID_ROWS) for c_idx in range(GRID_COLS) 
                          if r_idx % 3 == 0 and c_idx % 3 == 0)
    
    for alist in [ax.lines[num_static_lines:], ax.patches[num_static_patches:], ax.texts[num_static_texts:]]:
        while alist:
            alist.pop(0).remove()
    
    # Get sensor data directly in visualization
    try:
        raw_sv = [s.getValue() for s in gs_wb]
        ldf_local = [1 if v < LINE_THRESHOLD else 0 for v in raw_sv]
    except:
        ldf_local = [0, 0, 0]  # Default to no line detected
        raw_sv = [0, 0, 0]
    
    # Get robot display position (centered on line when sensors detect line)
    display_x, display_z = get_line_centered_position(rwp, crgp, ldf_local)
    
    # Update robot trail with display position
    robot_trail_world.append((display_x, display_z))
    if len(robot_trail_world) > 200:
        robot_trail_world.pop(0)
    
    if len(robot_trail_world) > 1:
        tx, tz = zip(*robot_trail_world)
        ax.plot(tx, tz, 'cyan', lw=2, alpha=0.7)
    
    # Draw planned path
    if path_esp and len(path_esp) > 1:
        planned_path_grid = path_esp
        pwc = [grid_to_world_center(r, c) for r, c in planned_path_grid]
        if pwc:
            px, pz = zip(*pwc)
            ax.plot(px, pz, 'mo--', lw=2, ms=5, alpha=0.8)
            if px:
                ax.plot(px[0], pz[0], 'm^', ms=8)  # Start
                ax.plot(px[-1], pz[-1], 'm*', ms=8)  # End
    
    # Draw current robot position (using display position when on line)
    ax.plot(display_x, display_z, 'ro', ms=10, mec='darkred', mew=1)
    
    # Draw robot orientation arrow from display position
    arr_len = GRID_CELL_SIZE * 0.8
    dx = arr_len * math.cos(rwp['theta'])
    dz = arr_len * math.sin(rwp['theta'])
    arrow = plt.matplotlib.patches.FancyArrowPatch(
        (display_x, display_z), (display_x + dx, display_z + dz),
        arrowstyle='->', mutation_scale=15, color='darkred', lw=2
    )
    ax.add_patch(arrow)
    
    # Highlight current grid cell with sensor-based color
    if crgp:
        wcx_curr, wcz_curr = grid_to_world_center(crgp[0], crgp[1])
        sensors_on_line = any(ldf_local)
        
        # Color based on SENSORS, not world_grid!
        highlight_color = 'green' if sensors_on_line else 'yellow'
        highlight_alpha = 0.5 if sensors_on_line else 0.3
        
        highlight_rect = plt.Rectangle(
            (wcx_curr - GRID_CELL_SIZE/2, wcz_curr - GRID_CELL_SIZE/2),
            GRID_CELL_SIZE, GRID_CELL_SIZE,
            edgecolor=highlight_color, facecolor=highlight_color, 
            alpha=highlight_alpha, linewidth=3
        )
        ax.add_patch(highlight_rect)
        
        # Debug info for current cell - show SENSOR status, not grid value
        if hasattr(ax, '_debug_text_handle_crgp_val') and ax._debug_text_handle_crgp_val in ax.texts:
            ax._debug_text_handle_crgp_val.remove()
        
        # Show what sensors say vs what grid says
        grid_val = world_grid[crgp[0]][crgp[1]] if (0 <= crgp[0] < GRID_ROWS and 0 <= crgp[1] < GRID_COLS) else 'OOB'
        sensor_status = "BLACK LINE" if sensors_on_line else "NO LINE"
        grid_says = "black" if grid_val == 0 else "white" if grid_val == 1 else "OOB"
        
        status_text = f"SENSORS: {sensor_status}\nGrid says: {grid_says}"
        status_color = 'green' if sensors_on_line else 'orange'
        
        ax._debug_text_handle_crgp_val = ax.text(
            wcx_curr, wcz_curr + GRID_CELL_SIZE * 0.6,
            status_text, ha='center', va='bottom',
            fontsize=8, color=status_color, weight='bold',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8)
        )

    # Draw goal
    gwx, gwz = grid_to_world_center(GOAL_ROW, GOAL_COL)
    ax.plot(gwx, gwz, 'g*', ms=15, mec='darkgreen', mew=1.5)
    
    # Info text - emphasize sensor readings
    sensors_on_line = any(ldf_local)
    centering_active = (display_x != rwp['x'] or display_z != rwp['z'])
    
    # Sensor-based status
    line_status = "✅ ON BLACK LINE (Sensors)" if sensors_on_line else "❌ NO LINE DETECTED"
    
    info = (f"Grid: {crgp} -> ({GOAL_ROW},{GOAL_COL}) | {line_status}\n"
           f"Actual: X={rwp['x']:.3f}, Z={rwp['z']:.3f}, θ={math.degrees(rwp['theta']):.1f}°\n"
           f"Display: X={display_x:.3f}, Z={display_z:.3f} | Centering: {centering_active}\n"
           f"Sensors (L,C,R): {ldf_local} | Raw: {[f'{v:.0f}' for v in raw_sv]}\n"
           f"Turn Phase: {webots_internal_turn_phase}")
    
    if hasattr(ax, '_info_text_handle') and ax._info_text_handle in ax.texts:
        ax._info_text_handle.remove()
    
    # Color code info box based on sensor status
    info_bg_color = 'lightgreen' if sensors_on_line else 'lightcoral'
    
    ax._info_text_handle = ax.text(
        0.02, 0.98, info, transform=ax.transAxes, va='top', fontsize=9,
        bbox=dict(boxstyle='round,pad=0.4', facecolor=info_bg_color, alpha=0.8)
    )
    
    plt.draw()
    plt.pause(0.001)

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Robot state
rwp = {'x': 0., 'z': 0., 'theta': 0.}
ple = 0.
pre = 0.
frod = True

# Motors and encoders
lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
le = robot.getDevice('left wheel sensor')
re = robot.getDevice('right wheel sensor')

for m in [lm, rm]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

for e in [le, re]:
    e.enable(timestep)

# Ground sensors
gs_wb = []
sens_names = ['gs0', 'gs1', 'gs2']
for name in sens_names:
    s = robot.getDevice(name)
    s.enable(timestep)
    gs_wb.append(s)

# Network variables
client_socket = None
is_connected_to_esp32 = False
esp32_command_state = 'stop'

def connect_to_esp32_server():
    global client_socket, is_connected_to_esp32
    print(f"Attempting ESP connection to {ESP32_IP_ADDRESS}...")
    try:
        if client_socket:
            client_socket.close()
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(2.0)
        client_socket.connect((ESP32_IP_ADDRESS, ESP32_PORT))
        client_socket.settimeout(0.05)
        is_connected_to_esp32 = True
        print(f"✅ ESP Connected!")
        return True
    except Exception as e:
        print(f"🔌 ESP Fail: {e}")
        is_connected_to_esp32 = False
        client_socket = None
        return False

# CRITICAL: Set initial position to match your actual robot starting position in Webots
# You may need to adjust these values based on your world setup
INITIAL_GRID_ROW, INITIAL_GRID_COL = 0, 16

# Calculate initial world position from grid coordinates
rwp['x'], rwp['z'] = grid_to_world_center(INITIAL_GRID_ROW, INITIAL_GRID_COL)
rwp['theta'] = math.pi / 2.0  # Facing down initially

# Verify the conversion works both ways
crgp = world_to_grid(rwp['x'], rwp['z'])
print(f"🚀 Robot init @ grid {crgp}, World (X={rwp['x']:.3f}, Z={rwp['z']:.3f}). Goal: ({GOAL_ROW},{GOAL_COL})")
print(f"🔍 Grid-to-World-to-Grid test: {INITIAL_GRID_ROW, INITIAL_GRID_COL} -> {rwp['x']:.3f}, {rwp['z']:.3f} -> {crgp}")

# COORDINATE ALIGNMENT DEBUG
print("\n🔍 COORDINATE ALIGNMENT TEST:")
print("Testing conversion for key grid positions...")
test_positions = [(0,0), (0,16), (14,0), (7,8), (GRID_ROWS-1, GRID_COLS-1)]
for row, col in test_positions:
    wx, wz = grid_to_world_center(row, col)
    back_row, back_col = world_to_grid(wx, wz)
    grid_val = world_grid[row][col] if (0 <= row < GRID_ROWS and 0 <= col < GRID_COLS) else 'OOB'
    grid_type = "BLACK LINE" if grid_val == 0 else "WHITE SPACE" if grid_val == 1 else "OUT OF BOUNDS"
    print(f"  Grid ({row:2},{col:2}) -> World ({wx:6.3f},{wz:6.3f}) -> Grid ({back_row:2},{back_col:2}) | Grid says: {grid_type}")

print(f"\nGrid Origin: X={GRID_ORIGIN_X}, Z={GRID_ORIGIN_Z}")
print(f"Grid Size: {GRID_ROWS}x{GRID_COLS}, Cell Size: {GRID_CELL_SIZE}")
print(f"World Bounds: X=[{GRID_ORIGIN_X:.3f}, {GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE:.3f}], "
      f"Z=[{GRID_ORIGIN_Z:.3f}, {GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE:.3f}]")
print("\n⚠️  If robot position doesn't match reality, adjust GRID_ORIGIN_X and GRID_ORIGIN_Z!")
print("=" * 80 + "\n")

# Main loop variables
loop_iteration_count = 0
last_esp_connection_attempt_time = 0
last_data_transmission_time = 0

# Main control loop
while robot.step(timestep) != -1:
    if loop_iteration_count == 0:
        connect_to_esp32_server()
        update_visualization(rwp, crgp, planned_path_grid)
    
    loop_iteration_count += 1
    current_simulation_time = robot.getTime()

    # Update robot position from odometry
    if not frod:
        lev, rev = le.getValue(), re.getValue()
        dd = ((lev - ple) * WHEEL_RADIUS + (rev - pre) * WHEEL_RADIUS) / 2.0
        dt = ((rev - pre) * WHEEL_RADIUS - (lev - ple) * WHEEL_RADIUS) / AXLE_LENGTH
        rwp['x'] += dd * math.cos(rwp['theta'] + dt / 2.0)
        rwp['z'] += dd * math.sin(rwp['theta'] + dt / 2.0)
        rwp['theta'] = math.atan2(math.sin(rwp['theta'] + dt), math.cos(rwp['theta'] + dt))
        ple, pre = lev, rev
    else:
        ple, pre = le.getValue(), re.getValue()
        frod = False
    
    # Update current grid position
    new_crgp = world_to_grid(rwp['x'], rwp['z'])
    if new_crgp != crgp:
        # Read sensors to show actual status
        raw_sv_debug = [s.getValue() for s in gs_wb]
        ldf_debug = [1 if v < LINE_THRESHOLD else 0 for v in raw_sv_debug]
        sensor_status = "SENSORS DETECT LINE" if any(ldf_debug) else "NO LINE DETECTED"
        
        cell_value = world_grid[new_crgp[0]][new_crgp[1]] if (0 <= new_crgp[0] < GRID_ROWS and 0 <= new_crgp[1] < GRID_COLS) else 'OUT OF BOUNDS'
        grid_says = "black line" if cell_value == 0 else "white space" if cell_value == 1 else "out of bounds"
        
        print(f"🤖 Robot moved to grid {new_crgp}. {sensor_status} (Grid map says: {grid_says})")
    crgp = new_crgp
        
    # Read ground sensors
    raw_sv = [s.getValue() for s in gs_wb]
    ldf = [1 if v < LINE_THRESHOLD else 0 for v in raw_sv]
    ols, ocs, ors = ldf

    # Handle ESP32 connection
    if not is_connected_to_esp32:
        if current_simulation_time - last_esp_connection_attempt_time > 3.0:
            connect_to_esp32_server()
            last_esp_connection_attempt_time = current_simulation_time
        lm.setVelocity(0.0)
        rm.setVelocity(0.0)
        if loop_iteration_count % 10 == 0:
            update_visualization(rwp, crgp, planned_path_grid)
        continue

    # Send data to ESP32
    if current_simulation_time - last_data_transmission_time > 0.1:
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
                'sensors_binary': ldf
            }
            client_socket.sendall((json.dumps(data) + '\n').encode('utf-8'))
            last_data_transmission_time = current_simulation_time
        except Exception as e:
            print(f"❌ ESP Send Error: {e}")
            is_connected_to_esp32 = False
            if client_socket:
                try:
                    client_socket.close()
                except Exception as e_close:
                    print(f"Err closing socket (send): {e_close}")
                client_socket = None
            continue
    
    # Receive commands from ESP32
    try:
        resp_b = client_socket.recv(1024)
        if resp_b:
            for msg_p in resp_b.decode('utf-8').strip().split('\n'):
                if not msg_p.strip():
                    continue
                try:
                    data_esp = json.loads(msg_p)
                    if data_esp.get('type') == 'esp32_command':
                        new_cmd = data_esp.get('action', 'stop')
                        if (new_cmd != esp32_command_state and 
                            esp32_command_state in ['turn_left', 'turn_right'] and 
                            new_cmd not in ['turn_left', 'turn_right']):
                            webots_internal_turn_phase = 'NONE'
                            webots_turn_command_active = None
                        esp32_command_state = new_cmd
                        planned_path_grid = data_esp.get('path', planned_path_grid)
                except json.JSONDecodeError as e:
                    print(f"⚠️ JSON ESP: '{msg_p}', {e}")
                except Exception as e:
                    print(f"⚠️ Proc ESP: {e}")
    except socket.timeout:
        pass
    except Exception as e:
        print(f"❌ ESP Receive Error: {e}")
        is_connected_to_esp32 = False
        if client_socket:
            try:
                client_socket.close()
            except Exception as e_close:
                print(f"Err closing socket (recv): {e_close}")
            client_socket = None
        continue

    # Motor control logic
    ls, rs = 0.0, 0.0

    if esp32_command_state not in ['turn_left', 'turn_right'] and webots_internal_turn_phase != 'NONE':
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None

    if esp32_command_state == 'stop':
        ls, rs = 0.0, 0.0
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None
        
    elif esp32_command_state == 'forward':
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None
        base_speed = FORWARD_SPEED
        
        # Line following logic with aggressive centering
        if not ols and ocs and not ors:     # Center sensor only
            ls, rs = base_speed, base_speed
        elif ols and ocs and not ors:       # Left + center
            ls, rs = base_speed - MODERATE_CORRECTION_DIFFERENTIAL, base_speed
        elif not ols and ocs and ors:       # Center + right
            ls, rs = base_speed, base_speed - MODERATE_CORRECTION_DIFFERENTIAL
        elif ols and not ocs and not ors:   # Left only
            ls, rs = base_speed - AGGRESSIVE_CORRECTION_DIFFERENTIAL, base_speed
        elif not ols and not ocs and ors:   # Right only
            ls, rs = base_speed, base_speed - AGGRESSIVE_CORRECTION_DIFFERENTIAL
        elif ols and ocs and ors:           # All sensors
            ls, rs = base_speed * 0.7, base_speed * 0.7
        elif not ols and not ocs and not ors: # No sensors
            ls, rs = base_speed * 0.2, base_speed * 0.2
        else:                               # Other combinations
            ls, rs = base_speed * 0.3, base_speed * 0.3

    elif esp32_command_state in ['turn_left', 'turn_right']:
        # Enhanced turn logic
        if (webots_turn_command_active != esp32_command_state or 
            webots_internal_turn_phase == 'NONE'):
            webots_turn_command_active = esp32_command_state
            webots_internal_turn_phase = 'INITIATE_SPIN'
            turn_phase_start_time = current_simulation_time
            print(f"Webots: Turn '{esp32_command_state}' -> INITIATE_SPIN")

        if webots_internal_turn_phase == 'INITIATE_SPIN':
            spin_in = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.7
            spin_out = FORWARD_SPEED * TURN_SPEED_FACTOR * 1.0
            ls, rs = (spin_in, spin_out) if webots_turn_command_active == 'turn_left' else (spin_out, spin_in)
            
            if current_simulation_time - turn_phase_start_time > MIN_INITIAL_SPIN_DURATION:
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_simulation_time
                print(f"Webots: Turn '{webots_turn_command_active}' -> SEARCHING_LINE")
                
        elif webots_internal_turn_phase == 'SEARCHING_LINE':
            srch_in = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.4
            srch_out = FORWARD_SPEED * TURN_SPEED_FACTOR * 0.8
            ls, rs = (srch_in, srch_out) if webots_turn_command_active == 'turn_left' else (srch_out, srch_in)
            
            acquired = (ocs or (ols and ocs) or (ors and ocs) or 
                       (webots_turn_command_active == 'turn_left' and ols and not ors) or 
                       (webots_turn_command_active == 'turn_right' and ors and not ols))
            
            if acquired:
                webots_internal_turn_phase = 'ADJUSTING_ON_LINE'
                turn_phase_start_time = current_simulation_time
                print(f"Webots: Turn '{webots_turn_command_active}' -> ADJUSTING_ON_LINE (Sens: {ldf})")
            elif current_simulation_time - turn_phase_start_time > MAX_SEARCH_SPIN_DURATION:
                print(f"Webots: Turn '{webots_turn_command_active}' -> TIMEOUT SEARCH. Abort local.")
                webots_internal_turn_phase = 'NONE'
                ls, rs = 0, 0
                
        elif webots_internal_turn_phase == 'ADJUSTING_ON_LINE':
            base = TURN_ADJUST_BASE_SPEED
            mod_diff_adj = MODERATE_CORRECTION_DIFFERENTIAL * (base / FORWARD_SPEED if FORWARD_SPEED > 1e-3 else 0.1)
            agg_diff_adj = AGGRESSIVE_CORRECTION_DIFFERENTIAL * (base / FORWARD_SPEED if FORWARD_SPEED > 1e-3 else 0.1)
            
            if not ols and ocs and not ors:
                ls, rs = base * 0.5, base * 0.5
            elif ols and ocs and not ors:
                ls, rs = base - mod_diff_adj, base
            elif not ols and ocs and ors:
                ls, rs = base, base - mod_diff_adj
            elif ols and not ocs and not ors:
                ls, rs = base - agg_diff_adj, base
            elif not ols and not ocs and ors:
                ls, rs = base, base - agg_diff_adj
            elif not any(ldf):
                print(f"Webots: Turn '{webots_turn_command_active}' Lost line in ADJUST -> SEARCH")
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_simulation_time
            else:
                ls, rs = base * 0.7, base * 0.7
                
            if current_simulation_time - turn_phase_start_time > MAX_ADJUST_DURATION:
                print(f"Webots: Turn '{webots_turn_command_active}' -> TIMEOUT ADJUST. Abort local.")
                webots_internal_turn_phase = 'NONE'
                ls, rs = 0, 0
    else:
        ls, rs = 0, 0
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None

    # Apply motor velocities
    lm.setVelocity(ls)
    rm.setVelocity(rs)
    
    # Update visualization periodically
    if loop_iteration_count % 3 == 0:
        update_visualization(rwp, crgp, planned_path_grid)
    
    # Status logging with sensor emphasis
    if loop_iteration_count % 25 == 0:
        cok_s = "🟢 ESP OK" if is_connected_to_esp32 else "🔴 ESP D/C"
        path_s = len(planned_path_grid) if planned_path_grid else "N/A"
        sensor_status = "✅ ON LINE" if any(ldf) else "❌ NO LINE"
        print(f"Sim: {current_simulation_time:.1f}s | {cok_s} | ESP: {esp32_command_state.upper()} | "
              f"Grid: {crgp} | Path: {path_s} | {sensor_status} {ldf} | Turn: {webots_internal_turn_phase}")

# Cleanup
if client_socket:
    try:
        client_socket.close()
    except:
        pass

if fig:
    print("🎨 Sim ended. Close plot window to exit.")
    plt.ioff()
    plt.show(block=True)

print("✅ Webots Controller Finished.")