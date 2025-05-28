# line_following_wifi_HIL.py (Webots Controller - Grid Navigation Client - V6 for 13x17 Grid with Obstacle Detection)

from controller import Robot, Motor, PositionSensor, DistanceSensor # Added DistanceSensor
import socket
import time
import math
import matplotlib.pyplot as plt
import numpy as np # For math operations if needed

# --- Network Configuration ---
ESP32_IP_ADDRESS = "192.168.53.193"  # <<<<<<<<<<< SET ESP32's ACTUAL IP (example)
ESP32_PORT = 8266
# ---------------------------

# --- Robot Parameters ---
WHEEL_RADIUS = 0.0205  # Standard e-puck wheel radius in meters
AXLE_LENGTH = 0.052    # Distance between the centers of the two wheels in meters (L)

# --- Grid Map Configuration (CRITICAL: Calibrate these to your Webots world) ---
GRID_ROWS = 13 # Must match ESP32
GRID_COLS = 17 # Must match ESP32
GRID_CELL_SIZE = 0.05  # Example: each grid cell is 5cm x 5cm in Webots

# The origin coordinates define where the CORNER of cell (0,0) is located in the Webots world.
# For a bottom-left start configuration:
# If (0,0) is the bottom-left cell, GRID_ORIGIN_X/Z is its bottom-left corner.
GRID_ORIGIN_X = -0.40  # X-coordinate of the bottom-left corner of cell (0,0)
GRID_ORIGIN_Z = -0.30   # Z-coordinate of the bottom-left corner of cell (0,0) (Y in Webots 2D view)

# Fine-tune offsets if robot position isn't mapping correctly to the grid cells.
# These are added to the GRID_ORIGIN to get the effective origin.
X_OFFSET = 2.0   # Fine-tune X offset if needed
Z_OFFSET = 2.0   # Fine-tune Z offset if needed (Y in Webots 2D view)

# Set to True to run in calibration mode (shows grid cells and coordinates on plot)
CALIBRATION_MODE = True
# --- Goal for this Webots instance (sent to ESP32) ---
# This is the target the robot will try to reach.
WEBOTS_TARGET_GOAL_ROW = 12
WEBOTS_TARGET_GOAL_COL = 16  # Example: Top-rightmost navigable cell
# -----------------------------------------------------------------------

# --- Robot Control Configuration ---
MAX_SPEED_CELL_NAV = 1.28   # Speed for moving towards next cell center (rad/s for motor)
TURN_SPEED_CELL_NAV = 1.5  # Speed for rotating to face next cell (rad/s for motor)
DIST_TO_CELL_THRESHOLD = GRID_CELL_SIZE * 0.20 # How close to get to cell center
ANGLE_TO_CELL_THRESHOLD = 0.01 # Radians (approx 5.7 degrees) for alignment
# -------------------------------------------------------------

# --- Obstacle Detection Configuration ---
PROXIMITY_SENSOR_NAMES = ['ps0', 'ps7', 'ps1', 'ps6'] # Front-facing sensors
# PROXIMITY_SENSOR_NAMES = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7'] # All sensors
PROXIMITY_THRESHOLD = 200  # Sensor value above which an obstacle is considered detected (adjust this!)
OBSTACLE_PROJECTION_DISTANCE = GRID_CELL_SIZE * 0.6 # How far in front of robot to check for obstacle cell
OBSTACLE_REPORT_COOLDOWN = 2.0 # Seconds before reporting the same obstacle cell again
last_obstacle_report_time = {} # Dictionary to store { (r,c): timestamp }
# -------------------------------------------------------------

# --- Matplotlib Live Plotting ---
plt.ion()
fig_map, ax_map = None, None
robot_path_trail_plot = [] # Stores (x,z) tuples of robot's path
current_planned_path_nodes_grid = [] # Stores (r,c) tuples of ESP32's path
plot_dynamic_artists = [] # List to keep track of artists to remove and redraw

# This MUST match the ESP32's initial world_grid for plotting obstacles correctly
# This will be updated if Webots tells ESP32 about a new obstacle
webots_side_world_grid = [
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # Goal (12,16) should be 0
]
if 0 <= WEBOTS_TARGET_GOAL_ROW < GRID_ROWS and 0 <= WEBOTS_TARGET_GOAL_COL < GRID_COLS:
    webots_side_world_grid[WEBOTS_TARGET_GOAL_ROW][WEBOTS_TARGET_GOAL_COL] = 0
# Also ensure the common (12,16) is 0 if it's different from the specific goal for some reason
if 0 <= 12 < GRID_ROWS and 0 <= 16 < GRID_COLS:
    webots_side_world_grid[12][16] = 0


def world_to_grid_cell(world_x, world_z):
    """
    Convert Webots world coordinates (x, z) to grid cell coordinates (row, col).
    Assumes GRID_ORIGIN_X/Z is the bottom-left CORNER of cell (0,0).
    Row 0 is at the 'bottom' (smallest Z for positive GRID_CELL_SIZE).
    Col 0 is at the 'left' (smallest X for positive GRID_CELL_SIZE).
    """
    # Effective origin after applying offsets
    eff_origin_x = GRID_ORIGIN_X + X_OFFSET
    eff_origin_z = GRID_ORIGIN_Z + Z_OFFSET

    # Calculate column: distance from origin / cell size
    col = int(round((world_x - eff_origin_x) / GRID_CELL_SIZE))
    
    # Calculate row: distance from origin / cell size
    # If positive Z is "up" on the map, and row 0 is at the bottom.
    row = int(round((world_z - eff_origin_z) / GRID_CELL_SIZE))

    # Clamp to grid boundaries
    clamped_row = max(0, min(row, GRID_ROWS - 1))
    clamped_col = max(0, min(col, GRID_COLS - 1))

    if (clamped_row != row or clamped_col != col) and webots_loop_counter % 100 == 1:
        print(f"Warning: Grid pos clamped from ({row},{col}) to ({clamped_row},{clamped_col}). World:({world_x:.3f},{world_z:.3f})")
    
    return clamped_row, clamped_col

def grid_cell_to_world_center(row, col):
    """
    Convert grid cell coordinates (row, col) to Webots world coordinates (x, z)
    representing the CENTER of that cell.
    """
    eff_origin_x = GRID_ORIGIN_X + X_OFFSET
    eff_origin_z = GRID_ORIGIN_Z + Z_OFFSET

    # Center of cell: origin_of_cell_corner + half_cell_size
    world_x_center = eff_origin_x + (col + 0.5) * GRID_CELL_SIZE
    world_z_center = eff_origin_z + (row + 0.5) * GRID_CELL_SIZE
    
    return world_x_center, world_z_center

def update_live_map_plot_grid(current_odom_pose_dict, current_grid_cell_tuple_rc, planned_path_grid_rc_tuples=None, obstacle_grid_data=None):
    """
    Updates the live Matplotlib plot for grid navigation.
    Args:
        current_odom_pose_dict (dict): {'x': float, 'y': float (world_z), 'theta': float}
        current_grid_cell_tuple_rc (tuple): (row, col) of the robot's current cell.
        planned_path_grid_rc_tuples (list of tuples): Path from ESP32 [(r,c), ...].
        obstacle_grid_data (list of lists): The grid used for plotting obstacles.
    """
    global fig_map, ax_map, robot_path_trail_plot, plot_dynamic_artists, webots_loop_counter

    eff_plot_origin_x = GRID_ORIGIN_X + X_OFFSET
    eff_plot_origin_z = GRID_ORIGIN_Z + Z_OFFSET

    if fig_map is None: # Initialize plot
        fig_map, ax_map = plt.subplots(figsize=(GRID_COLS*0.6, GRID_ROWS*0.6 + 1)) # Adjusted figure size
        ax_map.set_aspect('equal', 'box')
        ax_map.set_xlabel("World X (m)")
        ax_map.set_ylabel("World Z (m) (Webots Y)")
        title_str = (f"Robot Grid Nav - Cell:(0,0) Corner at ({eff_plot_origin_x:.2f}, {eff_plot_origin_z:.2f})\n"
                     f"Cell Size: {GRID_CELL_SIZE:.2f}m. Offsets: (X:{X_OFFSET:.2f}, Z:{Z_OFFSET:.2f})")
        ax_map.set_title(title_str, fontsize=10)

        # Draw grid lines
        for r_idx_plot in range(GRID_ROWS + 1): # Horizontal lines
            wz = eff_plot_origin_z + r_idx_plot * GRID_CELL_SIZE
            ax_map.plot([eff_plot_origin_x, eff_plot_origin_x + GRID_COLS * GRID_CELL_SIZE],
                        [wz, wz], 'k-', alpha=0.2, lw=0.7, zorder=0)
        for c_idx_plot in range(GRID_COLS + 1): # Vertical lines
            wx = eff_plot_origin_x + c_idx_plot * GRID_CELL_SIZE
            ax_map.plot([wx, wx],
                        [eff_plot_origin_z, eff_plot_origin_z + GRID_ROWS * GRID_CELL_SIZE],
                        'k-', alpha=0.2, lw=0.7, zorder=0)
        
        # If your Z (Webots Y) increases "downwards" on the plot, you might need to invert.
        # Default Matplotlib Y increases upwards. If row 0 is at smallest Z, this should be fine.
        # Test: if map appears upside down (row 0 at top of plot), uncomment:
        # ax_map.invert_yaxis() 
        update_live_map_plot_grid.static_artists_drawn = True

        if CALIBRATION_MODE:
            for r_cal in range(GRID_ROWS):
                for c_cal in range(GRID_COLS):
                    if r_cal % 2 == 0 and c_cal % 2 == 0: # Label subset of cells
                        wx_text, wz_text = grid_cell_to_world_center(r_cal, c_cal)
                        ax_map.text(wx_text, wz_text, f"({r_cal},{c_cal})", ha='center', va='center',
                                    fontsize=6, color='darkblue', alpha=0.7, zorder=3)
            # Mark effective origin (corner of 0,0)
            ax_map.plot(eff_plot_origin_x, eff_plot_origin_z, 'rX', ms=8, mew=1.5, label='Cell (0,0) Corner')
            ax_map.text(eff_plot_origin_x, eff_plot_origin_z, " (0,0) Corner", color='red', fontsize=8, ha='left', va='bottom')

        # Plot initial obstacles from webots_side_world_grid
        if obstacle_grid_data:
            for r_obs in range(len(obstacle_grid_data)):
                for c_obs in range(len(obstacle_grid_data[0])):
                    if obstacle_grid_data[r_obs][c_obs] == 1:
                        wx_center_obs, wz_center_obs = grid_cell_to_world_center(r_obs, c_obs)
                        rect_obs = plt.Rectangle((wx_center_obs - GRID_CELL_SIZE/2, wz_center_obs - GRID_CELL_SIZE/2),
                                             GRID_CELL_SIZE, GRID_CELL_SIZE,
                                             fc='dimgray', alpha=0.6, zorder=1)
                        ax_map.add_patch(rect_obs)
        update_live_map_plot_grid.initial_obstacles_drawn = True


    # Remove old dynamic artists
    for artist in plot_dynamic_artists:
        artist.remove()
    plot_dynamic_artists = []

    # Robot odometry trail
    robot_path_trail_plot.append((current_odom_pose_dict['x'], current_odom_pose_dict['y']))
    if len(robot_path_trail_plot) > 700: robot_path_trail_plot.pop(0) # Keep trail length manageable
    if len(robot_path_trail_plot) > 1:
        line_trail, = ax_map.plot(*zip(*robot_path_trail_plot), 'c-', lw=1.5, alpha=0.8, zorder=4, label='Odometry Trail')
        plot_dynamic_artists.append(line_trail)

    # Planned path from ESP32
    if planned_path_grid_rc_tuples and len(planned_path_grid_rc_tuples) > 1:
        path_world_coords = [grid_cell_to_world_center(r,c) for r,c in planned_path_grid_rc_tuples]
        path_world_x, path_world_z = zip(*path_world_coords)
        line_planned, = ax_map.plot(path_world_x, path_world_z, 'b*--', lw=1.5, ms=4, alpha=0.7, zorder=5, label='Planned Path (ESP32)')
        plot_dynamic_artists.append(line_planned)

    # Current robot position and orientation
    rx, rz_world, rth = current_odom_pose_dict['x'], current_odom_pose_dict['y'], current_odom_pose_dict['theta']
    dot_robot, = ax_map.plot(rx, rz_world, 'ro', ms=6, zorder=6, mec='black', label='Robot Position')
    plot_dynamic_artists.append(dot_robot)
    arrow_len = GRID_CELL_SIZE * 0.6
    arrow_robot = ax_map.arrow(rx, rz_world, arrow_len * math.cos(rth), arrow_len * math.sin(rth),
                               head_width=arrow_len*0.3, head_length=arrow_len*0.5,
                               fc='maroon', ec='maroon', zorder=7, lw=1)
    plot_dynamic_artists.append(arrow_robot)

    # Highlight current grid cell
    if current_grid_cell_tuple_rc:
        curr_cell_wx_center, curr_cell_wz_center = grid_cell_to_world_center(current_grid_cell_tuple_rc[0], current_grid_cell_tuple_rc[1])
        rect_curr_cell = plt.Rectangle((curr_cell_wx_center - GRID_CELL_SIZE/2, curr_cell_wz_center - GRID_CELL_SIZE/2),
                                    GRID_CELL_SIZE, GRID_CELL_SIZE,
                                    edgecolor='gold', facecolor='yellow', alpha=0.35, zorder=0, lw=1.5)
        ax_map.add_patch(rect_curr_cell) # Add_patch artists need to be tracked for removal if they change
        plot_dynamic_artists.append(rect_curr_cell)
    
    # Target goal cell marker
    if WEBOTS_TARGET_GOAL_ROW != -1:
        goal_wx, goal_wz = grid_cell_to_world_center(WEBOTS_TARGET_GOAL_ROW, WEBOTS_TARGET_GOAL_COL)
        goal_marker, = ax_map.plot(goal_wx, goal_wz, 'gP', ms=10, label='Target Goal', zorder=3, mec='darkgreen') # P for Plus
        plot_dynamic_artists.append(goal_marker)


    # Add legend if not already added (and if there's something to legend)
    if not hasattr(update_live_map_plot_grid, 'legend_added_flag'):
        handles, labels = ax_map.get_legend_handles_labels()
        if handles: # Only add legend if there are labeled artists
            # Place legend outside the plot area
            ax_map.legend(handles, labels, loc='upper left', fontsize='x-small', bbox_to_anchor=(1.02, 1.0))
            fig_map.tight_layout(rect=[0, 0, 0.82, 1]) # Adjust layout to make space for legend
            update_live_map_plot_grid.legend_added_flag = True
    
    # Set plot bounds (only once or periodically)
    if not hasattr(update_live_map_plot_grid, 'bounds_set_flag') or webots_loop_counter % 200 == 1:
        margin = GRID_CELL_SIZE * 1.0 # Margin around the grid
        min_wx_plot = eff_plot_origin_x - margin
        max_wx_plot = eff_plot_origin_x + GRID_COLS * GRID_CELL_SIZE + margin
        min_wz_plot = eff_plot_origin_z - margin
        max_wz_plot = eff_plot_origin_z + GRID_ROWS * GRID_CELL_SIZE + margin
        
        ax_map.set_xlim(min_wx_plot, max_wx_plot)
        # Check if y-axis was inverted when setting limits
        if hasattr(ax_map, 'yaxis_inverted') and ax_map.yaxis_inverted():
             ax_map.set_ylim(max_wz_plot, min_wz_plot) # Inverted
        else:
             ax_map.set_ylim(min_wz_plot, max_wz_plot) # Normal
        update_live_map_plot_grid.bounds_set_flag = True

    plt.draw()
    plt.pause(0.001)

# --- Initialize Robot & Devices ---
robot = Robot()
timestep = int(robot.getBasicTimeStep()) # [ms]

# Robot pose: x, y (world_z), theta (orientation)
robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
prev_left_enc_rad, prev_right_enc_rad = 0.0, 0.0
first_odom_run = True

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
if not (left_motor and right_motor): print("Critical: Motors missing"); exit(1)
for m in (left_motor, right_motor): m.setPosition(float('inf')); m.setVelocity(0.0)
print("✅ Webots: Motors initialized.")

# Encoders (PositionSensors)
left_enc = robot.getDevice('left wheel sensor')
right_enc = robot.getDevice('right wheel sensor')
encoders_ok = False
if left_enc and right_enc:
    left_enc.enable(timestep)
    right_enc.enable(timestep)
    encoders_ok = True
    print("✅ Webots: Encoders enabled.")
else:
    print("⚠️ Webots: Encoders missing. Odometry will not work.")

# Proximity Sensors
proximity_sensors = []
if PROXIMITY_SENSOR_NAMES:
    for name in PROXIMITY_SENSOR_NAMES:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        proximity_sensors.append(sensor)
    print(f"✅ Webots: Proximity sensors enabled: {PROXIMITY_SENSOR_NAMES}")
else:
    print("ℹ️ Webots: No proximity sensors specified.")


# --- Network Client ---
client_sock = None
connection_ok = False
last_connection_attempt_time = 0

def setup_client_connection():
    global client_sock, connection_ok, last_connection_attempt_time
    if client_sock:
        try: client_sock.close()
        except: pass
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_sock.settimeout(2.0) # Timeout for connection attempt
    print(f"Webots: Attempting to connect to ESP32 @ {ESP32_IP_ADDRESS}:{ESP32_PORT}...")
    try:
        client_sock.connect((ESP32_IP_ADDRESS, ESP32_PORT))
        client_sock.settimeout(0.1) # Shorter timeout for send/recv operations
        connection_ok = True
        print("✅ Webots: Connected to ESP32.")
    except Exception as e:
        print(f"⚠️ Webots: Connection to ESP32 failed: {e}")
        connection_ok = False
    last_connection_attempt_time = time.time()
    return connection_ok

setup_client_connection() # Initial connection attempt

# --- Main Loop ---
webots_loop_counter = 0
target_cell_r_from_esp, target_cell_c_from_esp = -1, -1 # Target cell received from ESP32
robot_is_moving_to_target_cell = False

# Define a consistent start position for the robot in the Webots world.
# This should correspond to a known grid cell, e.g., (0,0).
# Adjust these if your robot doesn't start at the world origin or faces differently.
# Example: Start at the center of grid cell (0,0) facing "up" the grid (positive Z in our coord system)
r_start_sim, c_start_sim = 0, 0
initial_world_x, initial_world_z = grid_cell_to_world_center(r_start_sim, c_start_sim)
initial_theta = math.pi / 2.0  # Facing positive Z (upwards on the map if Z is Y-axis of plot)

robot_pose['x'] = initial_world_x
robot_pose['y'] = initial_world_z # 'y' in robot_pose is world Z
robot_pose['theta'] = initial_theta
first_odom_run = True # Recalculate prev_enc values based on this new start
print(f"Webots: Initialized robot pose to world ({robot_pose['x']:.2f}, {robot_pose['y']:.2f}), Theta: {math.degrees(robot_pose['theta']):.1f} deg.")
print(f"Webots: This corresponds to grid cell ({r_start_sim},{c_start_sim}). Target Goal: ({WEBOTS_TARGET_GOAL_ROW},{WEBOTS_TARGET_GOAL_COL})")


while robot.step(timestep) != -1:
    webots_loop_counter += 1
    current_time = time.time()

    # --- Odometry Calculation ---
    if encoders_ok:
        current_left_enc_rad = left_enc.getValue()
        current_right_enc_rad = right_enc.getValue()

        if first_odom_run:
            prev_left_enc_rad = current_left_enc_rad
            prev_right_enc_rad = current_right_enc_rad
            first_odom_run = False
        else:
            # Calculate distance wheeled by each wheel
            delta_L_rad = current_left_enc_rad - prev_left_enc_rad
            delta_R_rad = current_right_enc_rad - prev_right_enc_rad
            
            dist_L = delta_L_rad * WHEEL_RADIUS
            dist_R = delta_R_rad * WHEEL_RADIUS

            prev_left_enc_rad = current_left_enc_rad
            prev_right_enc_rad = current_right_enc_rad

            # Compute change in robot pose
            delta_dist_center = (dist_L + dist_R) / 2.0
            delta_theta = (dist_R - dist_L) / AXLE_LENGTH

            # Update pose (using the more accurate formula for dX, dY)
            # If delta_theta is very small, treat as straight line to avoid division by zero with sinc
            if abs(delta_theta) < 1e-6: # Effectively straight
                delta_x_world = delta_dist_center * math.cos(robot_pose['theta'])
                delta_z_world = delta_dist_center * math.sin(robot_pose['theta'])
            else: # Turning
                radius_of_turn_path = delta_dist_center / delta_theta
                delta_x_world = radius_of_turn_path * (math.sin(robot_pose['theta'] + delta_theta) - math.sin(robot_pose['theta']))
                delta_z_world = radius_of_turn_path * (math.cos(robot_pose['theta']) - math.cos(robot_pose['theta'] + delta_theta))


            robot_pose['x'] += delta_x_world
            robot_pose['y'] += delta_z_world # 'y' in pose dict is world Z
            robot_pose['theta'] += delta_theta
            
            # Normalize theta to be within -pi to pi
            robot_pose['theta'] = math.atan2(math.sin(robot_pose['theta']), math.cos(robot_pose['theta']))

    # Map current world coordinates to grid cell
    current_r_mapped, current_c_mapped = world_to_grid_cell(robot_pose['x'], robot_pose['y'])

    if webots_loop_counter % 100 == 1: # Print occasionally for debugging
        print(f"Loop {webots_loop_counter}: Robot World ({robot_pose['x']:.3f}, {robot_pose['y']:.3f}, Th:{math.degrees(robot_pose['theta']):.1f}) -> Grid ({current_r_mapped}, {current_c_mapped})")

    # --- Network Communication with ESP32 ---
    if not connection_ok:
        if current_time - last_connection_attempt_time > 5.0: # Retry connection every 5s
            setup_client_connection()
        left_motor.setVelocity(0.0); right_motor.setVelocity(0.0) # Stop if not connected
        if webots_loop_counter % 10 == 1 : update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, webots_side_world_grid)
        continue # Skip rest of loop if not connected

    # --- Obstacle Detection and Reporting ---
    detected_obstacle_in_front_cell_rc = None
    if proximity_sensors:
        front_sensor_active = False
        max_prox_value = 0
        for sensor in proximity_sensors: # Check relevant front sensors
            val = sensor.getValue()
            if val > max_prox_value: max_prox_value = val # For debugging
            if val > PROXIMITY_THRESHOLD:
                front_sensor_active = True
                break
        
        if front_sensor_active:
            # Project a point in front of the robot to find the obstacle cell
            obstacle_check_x = robot_pose['x'] + OBSTACLE_PROJECTION_DISTANCE * math.cos(robot_pose['theta'])
            obstacle_check_z = robot_pose['y'] + OBSTACLE_PROJECTION_DISTANCE * math.sin(robot_pose['theta'])
            obs_r, obs_c = world_to_grid_cell(obstacle_check_x, obstacle_check_z)
            
            # Check if this cell was thought to be free and if cooldown passed
            if webots_side_world_grid[obs_r][obs_c] == 0: # If we thought it was free
                if (obs_r, obs_c) not in last_obstacle_report_time or \
                   current_time - last_obstacle_report_time.get((obs_r, obs_c), 0) > OBSTACLE_REPORT_COOLDOWN:
                    
                    detected_obstacle_in_front_cell_rc = (obs_r, obs_c)
                    print(f"Webots: Detected NEW obstacle at grid cell ({obs_r},{obs_c}). MaxProx: {max_prox_value:.0f}")
                    last_obstacle_report_time[(obs_r, obs_c)] = current_time
                    # Also update local grid map for plotting immediately
                    webots_side_world_grid[obs_r][obs_c] = 1
                    # Re-initialize plot if an obstacle changes (simplest way to update static obstacles)
                    if fig_map and hasattr(update_live_map_plot_grid, 'initial_obstacles_drawn'):
                        plt.close(fig_map) # Close current figure
                        fig_map, ax_map = None, None # Force reinitialization
                        update_live_map_plot_grid.legend_added_flag = False # Reset legend flag
                        update_live_map_plot_grid.bounds_set_flag = False   # Reset bounds flag
                        print("Webots: Plot re-initialized due to new obstacle.")


    # Send POS and potentially OBSTACLE message to ESP32
    # Always send POS, add OBSTACLE if new one detected
    pos_msg = f"POS:{current_r_mapped},{current_c_mapped},{WEBOTS_TARGET_GOAL_ROW},{WEBOTS_TARGET_GOAL_COL}\n"
    if detected_obstacle_in_front_cell_rc:
        obs_msg = f"OBSTACLE:{detected_obstacle_in_front_cell_rc[0]},{detected_obstacle_in_front_cell_rc[1]}\n"
        message_to_esp32 = obs_msg + pos_msg # Send obstacle first
        print(f"Webots TX: {obs_msg.strip()} and {pos_msg.strip()}")
    else:
        message_to_esp32 = pos_msg
        if webots_loop_counter % 50 == 1: print(f"Webots TX: {pos_msg.strip()}")


    try:
        client_sock.sendall(message_to_esp32.encode('utf-8'))
    except Exception as e:
        print(f"❌ Webots: Send Error: {e}. Disconnecting.")
        connection_ok = False
        if webots_loop_counter % 10 == 1 : update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, webots_side_world_grid)
        continue

    # Receive command from ESP32
    command_from_esp32 = ""
    try:
        data_bytes = client_sock.recv(256) # Buffer for path and command
        if not data_bytes:
            print("⚠️ Webots: ESP32 closed connection (recv empty).")
            connection_ok = False
        else:
            full_response_str = data_bytes.decode('utf-8')
            messages = full_response_str.splitlines()
            temp_goto_cmd_this_cycle = None

            for msg_part in messages:
                msg = msg_part.strip()
                if not msg: continue

                if msg.startswith("PATH_GRID:"):
                    try:
                        path_str_payload = msg.split(":", 1)[1]
                        current_planned_path_nodes_grid = [tuple(map(int, cell.split(','))) for cell in path_str_payload.split(';')]
                        # print(f"Webots RX Path: {current_planned_path_nodes_grid}")
                    except Exception as ep:
                        print(f"Webots Warn: Parse PATH_GRID fail: '{msg}', E:{ep}")
                        current_planned_path_nodes_grid = []
                elif msg.startswith("GOTO:"):
                    try:
                        coords_str = msg.split(":", 1)[1].split(',')
                        target_cell_r_from_esp, target_cell_c_from_esp = int(coords_str[0]), int(coords_str[1])
                        robot_is_moving_to_target_cell = True
                        temp_goto_cmd_this_cycle = msg # Prioritize this
                        # print(f"Webots RX GOTO: ({target_cell_r_from_esp},{target_cell_c_from_esp})")
                    except Exception as eg:
                        print(f"Webots Warn: Parse GOTO fail: '{msg}', E:{eg}")
                        robot_is_moving_to_target_cell = False
                elif msg == "stop":
                    robot_is_moving_to_target_cell = False
                    target_cell_r_from_esp, target_cell_c_from_esp = -1, -1
                    temp_goto_cmd_this_cycle = msg # Prioritize this
                    # print(f"Webots RX: stop")
                elif msg: # Catch any other unexpected messages
                    print(f"⚠️ Webots: Unhandled msg from ESP32: '{msg}'")
            
            if temp_goto_cmd_this_cycle: # If GOTO or STOP was received, use it
                command_from_esp32 = temp_goto_cmd_this_cycle


    except socket.timeout: # No data received from ESP32 within timeout
        if webots_loop_counter % 100 == 1: print("Webots: Timeout receiving from ESP32. Last cmd active.")
        pass # Keep current command_from_esp32 or movement state
    except Exception as e:
        print(f"❌ Webots: Recv Error: {e}. Disconnecting.")
        connection_ok = False
    
    if not connection_ok:
        if webots_loop_counter % 10 == 1 : update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, webots_side_world_grid)
        continue

    # --- Execute Movement Command ---
    left_speed_cmd, right_speed_cmd = 0.0, 0.0
    if robot_is_moving_to_target_cell and target_cell_r_from_esp != -1:
        target_world_x, target_world_z = grid_cell_to_world_center(target_cell_r_from_esp, target_cell_c_from_esp)
        
        delta_x_to_target = target_world_x - robot_pose['x']
        delta_z_to_target = target_world_z - robot_pose['y']
        
        distance_to_target_center = math.sqrt(delta_x_to_target**2 + delta_z_to_target**2)
        angle_to_target_center = math.atan2(delta_z_to_target, delta_x_to_target)
        
        angle_difference = math.atan2(math.sin(angle_to_target_center - robot_pose['theta']), 
                                      math.cos(angle_to_target_center - robot_pose['theta']))

        if distance_to_target_center > DIST_TO_CELL_THRESHOLD:
            if abs(angle_difference) > ANGLE_TO_CELL_THRESHOLD: # Need to turn
                # Proportional turn
                turn_effort = angle_difference * 20.0 # P-controller gain for turning (tune this)
                # Clamp turn speed
                turn_speed_val = max(-TURN_SPEED_CELL_NAV, min(TURN_SPEED_CELL_NAV, turn_effort * TURN_SPEED_CELL_NAV))
                left_speed_cmd = -turn_speed_val
                right_speed_cmd = turn_speed_val
            else: # Aligned, move forward
                left_speed_cmd = MAX_SPEED_CELL_NAV
                right_speed_cmd = MAX_SPEED_CELL_NAV
        else: # Arrived at the center of the target cell (or close enough)
            # print(f"Webots: Arrived at cell ({target_cell_r_from_esp},{target_cell_c_from_esp}). Waiting for next GOTO.")
            robot_is_moving_to_target_cell = False # Stop until next GOTO from ESP
            left_speed_cmd, right_speed_cmd = 0.0, 0.0
            # The ESP32 will send the next GOTO based on its path logic once POS confirms arrival
            
    elif command_from_esp32 == "stop": # Explicit stop command
        left_speed_cmd, right_speed_cmd = 0.0, 0.0
        robot_is_moving_to_target_cell = False
        # print("Webots: Executing STOP command.")

    left_motor.setVelocity(left_speed_cmd)
    right_motor.setVelocity(right_speed_cmd)

    # Update live plot (e.g., every 5 simulation steps)
    if encoders_ok and webots_loop_counter % 5 == 1: # Update plot less frequently
        update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, webots_side_world_grid)

# --- End of Simulation ---
if client_sock:
    try: client_sock.sendall("stop\n".encode('utf-8')) # Tell ESP32 to stop if simulation ends
    except: pass
    client_sock.close()
    print("ℹ️ Webots: Client socket closed.")

if fig_map: # Keep plot open after simulation ends
    print("ℹ️ Webots: Simulation ended. Plot window active. Close plot to exit.")
    plt.ioff()
    plt.show(block=True)

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
print("ℹ️ Webots: Controller finished.")
