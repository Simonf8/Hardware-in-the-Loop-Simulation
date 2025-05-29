from controller import Robot, Motor, PositionSensor, DistanceSensor
import socket
import time
import math
import matplotlib.pyplot as plt
import numpy as np

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
# !!!!!!!!!! CRITICAL: VERIFY THIS VALUE IN YOUR WEBOTS WORLD !!!!!!!!!!
GRID_CELL_SIZE = 0.081 # Example: each grid cell is 8.1cm x 8.1cm in Webots
# !!!!!!!!!! CRITICAL: VERIFY THESE COORDINATES IN YOUR WEBOTS WORLD !!!!!!!!!!
GRID_ORIGIN_X = -0.40  # X-coordinate of the bottom-left corner of cell (0,0)
GRID_ORIGIN_Z = -0.30   # Z-coordinate of the bottom-left corner of cell (0,0) (Y in Webots 2D view)

X_OFFSET = 0.0
Z_OFFSET = 0.0
CALIBRATION_MODE = True
WEBOTS_TARGET_GOAL_ROW = 12
WEBOTS_TARGET_GOAL_COL = 0
# -----------------------------------------------------------------------

# --- Robot Control Configuration ---
MAX_SPEED_CELL_NAV = 3.28 # Max speed for GOTO commands
TURN_SPEED_CELL_NAV = 2.8  # Max speed for turning during GOTO commands
DIST_TO_CELL_THRESHOLD = GRID_CELL_SIZE * 0.35 # How close to cell center to consider "near"
ANGLE_TO_CELL_THRESHOLD = 0.08 # Radians (approx 4.5 degrees) for alignment for forward motion during GOTO

# --- Line Sensor Configuration ---
LINE_THRESHOLD = 600  # Values BELOW this are considered "on the line" (typical for IR)
GROUND_SENSOR_NAMES = ['gs0', 'gs1', 'gs2'] # Left, Middle, Right

# --- Obstacle Detection Configuration ---
PROXIMITY_SENSOR_NAMES = ['ps0', 'ps7', 'ps1', 'ps6']
PROXIMITY_THRESHOLD = 200
OBSTACLE_PROJECTION_DISTANCE = GRID_CELL_SIZE * 0.6
OBSTACLE_REPORT_COOLDOWN = 2.0
last_obstacle_report_time = {}
# -------------------------------------------------------------

# --- Matplotlib Live Plotting ---
plt.ion()
fig_map, ax_map = None, None
robot_path_trail_plot = []
current_planned_path_nodes_grid = []
plot_dynamic_artists = []
webots_side_world_grid = [
    [1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],[1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0],[0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0],[0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1]
]
if 0 <= WEBOTS_TARGET_GOAL_ROW < GRID_ROWS and 0 <= WEBOTS_TARGET_GOAL_COL < GRID_COLS:
    webots_side_world_grid[WEBOTS_TARGET_GOAL_ROW][WEBOTS_TARGET_GOAL_COL] = 0
if 0 <= 12 < GRID_ROWS and (GRID_COLS -1) >=0 :
      webots_side_world_grid[12][GRID_COLS-1] = 0


def world_to_grid_cell(world_x, world_z):
    global webots_loop_counter 
    eff_origin_x = GRID_ORIGIN_X + X_OFFSET
    eff_origin_z = GRID_ORIGIN_Z + Z_OFFSET
    col = int(round((world_x - eff_origin_x) / GRID_CELL_SIZE))
    row = int(round((world_z - eff_origin_z) / GRID_CELL_SIZE))
    clamped_row = max(0, min(row, GRID_ROWS - 1))
    clamped_col = max(0, min(col, GRID_COLS - 1))
    if (clamped_row != row or clamped_col != col) and webots_loop_counter % 100 == 1:
        print(f"Warning: Grid pos clamped from ({row},{col}) to ({clamped_row},{clamped_col}). World:({world_x:.3f},{world_z:.3f})")
    return clamped_row, clamped_col

def grid_cell_to_world_center(row, col):
    eff_origin_x = GRID_ORIGIN_X + X_OFFSET
    eff_origin_z = GRID_ORIGIN_Z + Z_OFFSET
    world_x_center = eff_origin_x + (col + 0.5) * GRID_CELL_SIZE
    world_z_center = eff_origin_z + (row + 0.5) * GRID_CELL_SIZE
    return world_x_center, world_z_center

def update_live_map_plot_grid(current_odom_pose_dict, current_grid_cell_tuple_rc, planned_path_grid_rc_tuples=None, obstacle_grid_data=None):
    global fig_map, ax_map, robot_path_trail_plot, plot_dynamic_artists, webots_loop_counter
    eff_plot_origin_x = GRID_ORIGIN_X + X_OFFSET
    eff_plot_origin_z = GRID_ORIGIN_Z + Z_OFFSET
    if fig_map is None: 
        fig_map, ax_map = plt.subplots(figsize=(GRID_COLS*0.6, GRID_ROWS*0.6 + 1)) 
        ax_map.set_aspect('equal', 'box')
        ax_map.set_xlabel("World X (m)")
        ax_map.set_ylabel("World Z (m) (Webots Y)")
        title_str = (f"Robot Grid Nav - Cell:(0,0) Corner at ({eff_plot_origin_x:.2f}, {eff_plot_origin_z:.2f})\n"
                     f"Cell Size: {GRID_CELL_SIZE:.2f}m. Offsets: (X:{X_OFFSET:.2f}, Z:{Z_OFFSET:.2f})")
        ax_map.set_title(title_str, fontsize=10)
        for r_idx_plot in range(GRID_ROWS + 1): 
            wz = eff_plot_origin_z + r_idx_plot * GRID_CELL_SIZE
            ax_map.plot([eff_plot_origin_x, eff_plot_origin_x + GRID_COLS * GRID_CELL_SIZE],
                        [wz, wz], 'k-', alpha=0.2, lw=0.7, zorder=0)
        for c_idx_plot in range(GRID_COLS + 1): 
            wx = eff_plot_origin_x + c_idx_plot * GRID_CELL_SIZE
            ax_map.plot([wx, wx],
                        [eff_plot_origin_z, eff_plot_origin_z + GRID_ROWS * GRID_CELL_SIZE],
                        'k-', alpha=0.2, lw=0.7, zorder=0)
        if not hasattr(update_live_map_plot_grid, 'static_artists_drawn'):
            update_live_map_plot_grid.static_artists_drawn = False # type: ignore
        if not hasattr(update_live_map_plot_grid, 'initial_obstacles_drawn'):
            update_live_map_plot_grid.initial_obstacles_drawn = False # type: ignore
        if not hasattr(update_live_map_plot_grid, 'legend_added_flag'):
            update_live_map_plot_grid.legend_added_flag = False # type: ignore
        if not hasattr(update_live_map_plot_grid, 'bounds_set_flag'):
            update_live_map_plot_grid.bounds_set_flag = False # type: ignore

        update_live_map_plot_grid.static_artists_drawn = True # type: ignore
        if CALIBRATION_MODE:
            for r_cal in range(GRID_ROWS):
                for c_cal in range(GRID_COLS):
                    if r_cal % 2 == 0 and c_cal % 2 == 0: 
                        wx_text, wz_text = grid_cell_to_world_center(r_cal, c_cal)
                        ax_map.text(wx_text, wz_text, f"({r_cal},{c_cal})", ha='center', va='center',
                                    fontsize=6, color='darkblue', alpha=0.7, zorder=3)
            ax_map.plot(eff_plot_origin_x, eff_plot_origin_z, 'rX', ms=8, mew=1.5, label='Cell (0,0) Corner')
            ax_map.text(eff_plot_origin_x, eff_plot_origin_z, " (0,0) Corner", color='red', fontsize=8, ha='left', va='bottom')
        if obstacle_grid_data:
            for r_obs in range(len(obstacle_grid_data)):
                for c_obs in range(len(obstacle_grid_data[0])):
                    if obstacle_grid_data[r_obs][c_obs] == 1:
                        wx_center_obs, wz_center_obs = grid_cell_to_world_center(r_obs, c_obs)
                        rect_obs = plt.Rectangle((wx_center_obs - GRID_CELL_SIZE/2, wz_center_obs - GRID_CELL_SIZE/2),
                                                 GRID_CELL_SIZE, GRID_CELL_SIZE,
                                                 fc='dimgray', alpha=0.6, zorder=1)
                        ax_map.add_patch(rect_obs)
        update_live_map_plot_grid.initial_obstacles_drawn = True # type: ignore
    for artist in plot_dynamic_artists:
        artist.remove()
    plot_dynamic_artists = []
    robot_path_trail_plot.append((current_odom_pose_dict['x'], current_odom_pose_dict['y']))
    if len(robot_path_trail_plot) > 700: robot_path_trail_plot.pop(0) 
    if len(robot_path_trail_plot) > 1:
        line_trail, = ax_map.plot(*zip(*robot_path_trail_plot), 'c-', lw=1.5, alpha=0.8, zorder=4, label='Odometry Trail')
        plot_dynamic_artists.append(line_trail)
    if planned_path_grid_rc_tuples and len(planned_path_grid_rc_tuples) > 1:
        path_world_coords = [grid_cell_to_world_center(r,c) for r,c in planned_path_grid_rc_tuples]
        path_world_x, path_world_z = zip(*path_world_coords)
        line_planned, = ax_map.plot(path_world_x, path_world_z, 'b*--', lw=1.5, ms=4, alpha=0.7, zorder=5, label='Planned Path (ESP32)')
        plot_dynamic_artists.append(line_planned)
    rx, rz_world, rth = current_odom_pose_dict['x'], current_odom_pose_dict['y'], current_odom_pose_dict['theta']
    dot_robot, = ax_map.plot(rx, rz_world, 'ro', ms=6, zorder=6, mec='black', label='Robot Position')
    plot_dynamic_artists.append(dot_robot)
    arrow_len = GRID_CELL_SIZE * 0.6
    arrow_robot = ax_map.arrow(rx, rz_world, arrow_len * math.cos(rth), arrow_len * math.sin(rth),
                                 head_width=arrow_len*0.3, head_length=arrow_len*0.5,
                                 fc='maroon', ec='maroon', zorder=7, lw=1)
    plot_dynamic_artists.append(arrow_robot)
    if current_grid_cell_tuple_rc:
        curr_cell_wx_center, curr_cell_wz_center = grid_cell_to_world_center(current_grid_cell_tuple_rc[0], current_grid_cell_tuple_rc[1])
        rect_curr_cell = plt.Rectangle((curr_cell_wx_center - GRID_CELL_SIZE/2, curr_cell_wz_center - GRID_CELL_SIZE/2),
                                     GRID_CELL_SIZE, GRID_CELL_SIZE,
                                     edgecolor='gold', facecolor='yellow', alpha=0.35, zorder=0, lw=1.5)
        ax_map.add_patch(rect_curr_cell) 
        plot_dynamic_artists.append(rect_curr_cell)
    if WEBOTS_TARGET_GOAL_ROW != -1:
        goal_wx, goal_wz = grid_cell_to_world_center(WEBOTS_TARGET_GOAL_ROW, WEBOTS_TARGET_GOAL_COL)
        goal_marker, = ax_map.plot(goal_wx, goal_wz, 'gP', ms=10, label='Target Goal', zorder=3, mec='darkgreen') 
        plot_dynamic_artists.append(goal_marker)
    if not hasattr(update_live_map_plot_grid, 'legend_added_flag') or not update_live_map_plot_grid.legend_added_flag : # type: ignore
        handles, labels = ax_map.get_legend_handles_labels()
        if handles: 
            ax_map.legend(handles, labels, loc='upper left', fontsize='x-small', bbox_to_anchor=(1.02, 1.0))
            fig_map.tight_layout(rect=[0, 0, 0.82, 1]) 
            update_live_map_plot_grid.legend_added_flag = True # type: ignore
    if not hasattr(update_live_map_plot_grid, 'bounds_set_flag') or not update_live_map_plot_grid.bounds_set_flag or webots_loop_counter % 200 == 1: # type: ignore
        margin = GRID_CELL_SIZE * 1.0 
        min_wx_plot = eff_plot_origin_x - margin
        max_wx_plot = eff_plot_origin_x + GRID_COLS * GRID_CELL_SIZE + margin
        min_wz_plot = eff_plot_origin_z - margin
        max_wz_plot = eff_plot_origin_z + GRID_ROWS * GRID_CELL_SIZE + margin
        ax_map.set_xlim(min_wx_plot, max_wx_plot)
        if hasattr(ax_map, 'yaxis_inverted') and ax_map.yaxis_inverted(): # type: ignore
                ax_map.set_ylim(max_wz_plot, min_wz_plot) 
        else:
                ax_map.set_ylim(min_wz_plot, max_wz_plot) 
        update_live_map_plot_grid.bounds_set_flag = True # type: ignore
    plt.draw()
    plt.pause(0.001)


# --- Initialize Robot & Devices ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
prev_left_enc_rad, prev_right_enc_rad = 0.0, 0.0
first_odom_run = True

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
if not (left_motor and right_motor): print("Critical: Motors missing"); exit(1)
for m in (left_motor, right_motor): m.setPosition(float('inf')); m.setVelocity(0.0)
print("✅ Webots: Motors initialized.")

# Encoders
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

# Ground Sensors
ground_sensors = []
if len(GROUND_SENSOR_NAMES) == 3:
    for name in GROUND_SENSOR_NAMES:
        sensor = robot.getDevice(name)
        if sensor is None:
            print(f"⚠️ Warning: Ground sensor device '{name}' not found. Check Webots.")
        else:
            sensor.enable(timestep)
            ground_sensors.append(sensor)
            print(f"✅ Ground sensor '{name}' enabled.")
    if len(ground_sensors) != 3:
        print(f"❌ Error: Expected 3 ground sensors based on NAMES, but found {len(ground_sensors)}. Line data to ESP32 may be incomplete.")
else:
    print("❌ Error: GROUND_SENSOR_NAMES does not contain 3 sensor names. Ground sensors not initialized.")


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
    client_sock.settimeout(2.0) 
    print(f"Webots: Attempting to connect to ESP32 @ {ESP32_IP_ADDRESS}:{ESP32_PORT}...")
    try:
        client_sock.connect((ESP32_IP_ADDRESS, ESP32_PORT))
        client_sock.settimeout(0.1) 
        connection_ok = True
        print("✅ Webots: Connected to ESP32.")
    except Exception as e:
        print(f"⚠️ Webots: Connection to ESP32 failed: {e}")
        connection_ok = False
    last_connection_attempt_time = time.time()
    return connection_ok
setup_client_connection()

# --- Main Loop ---
webots_loop_counter = 0
target_cell_r_from_esp, target_cell_c_from_esp = -1, -1 # For GOTO commands
robot_is_moving_to_target_cell = False # Flag for GOTO navigation state
# Variables for direct speed commands from ESP32
esp_commanded_left_speed = 0.0
esp_commanded_right_speed = 0.0
use_esp_direct_speeds = False # Flag to indicate if ESP is controlling speeds directly

r_start_sim, c_start_sim = 0,16 # Default start cell from your log
initial_world_x, initial_world_z = grid_cell_to_world_center(r_start_sim, c_start_sim)
initial_theta = math.pi / 2.0  # Default: Facing positive Z (as per your log's 90 deg)

robot_pose['x'] = initial_world_x
robot_pose['y'] = initial_world_z
robot_pose['theta'] = initial_theta
first_odom_run = True
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
            delta_L_rad = current_left_enc_rad - prev_left_enc_rad
            delta_R_rad = current_right_enc_rad - prev_right_enc_rad
            dist_L = delta_L_rad * WHEEL_RADIUS
            dist_R = delta_R_rad * WHEEL_RADIUS
            prev_left_enc_rad = current_left_enc_rad
            prev_right_enc_rad = current_right_enc_rad
            delta_dist_center = (dist_L + dist_R) / 2.0
            delta_theta = (dist_R - dist_L) / AXLE_LENGTH
            if abs(delta_theta) < 1e-6: 
                delta_x_world = delta_dist_center * math.cos(robot_pose['theta'])
                delta_z_world = delta_dist_center * math.sin(robot_pose['theta'])
            else: 
                radius_of_turn_path = delta_dist_center / delta_theta
                delta_x_world = radius_of_turn_path * (math.sin(robot_pose['theta'] + delta_theta) - math.sin(robot_pose['theta']))
                delta_z_world = radius_of_turn_path * (math.cos(robot_pose['theta']) - math.cos(robot_pose['theta'] + delta_theta))
            robot_pose['x'] += delta_x_world
            robot_pose['y'] += delta_z_world 
            robot_pose['theta'] += delta_theta
            robot_pose['theta'] = math.atan2(math.sin(robot_pose['theta']), math.cos(robot_pose['theta']))

    current_r_mapped, current_c_mapped = world_to_grid_cell(robot_pose['x'], robot_pose['y'])

    # --- Read Ground Sensors ---
    s_left_on_line, s_middle_on_line, s_right_on_line = 0, 0, 0 
    if len(ground_sensors) == 3:
        raw_gs_values = [gs.getValue() for gs in ground_sensors]
        s_left_on_line = 1 if raw_gs_values[0] < LINE_THRESHOLD else 0
        s_middle_on_line = 1 if raw_gs_values[1] < LINE_THRESHOLD else 0
        s_right_on_line = 1 if raw_gs_values[2] < LINE_THRESHOLD else 0
        if webots_loop_counter % 20 == 1: 
            print(f"Debug GS Raw: L={raw_gs_values[0]:.0f} M={raw_gs_values[1]:.0f} R={raw_gs_values[2]:.0f} -> Binarized: L={s_left_on_line} M={s_middle_on_line} R={s_right_on_line}")
    elif webots_loop_counter % 100 == 1: 
        print("⚠️ Warning: Ground sensors not fully available for reading.")


    if CALIBRATION_MODE and webots_loop_counter % 100 == 1:
        print(f"Loop {webots_loop_counter}: Robot World ({robot_pose['x']:.3f}, {robot_pose['y']:.3f}, Th:{math.degrees(robot_pose['theta']):.1f}) -> Grid ({current_r_mapped}, {current_c_mapped}) | Target ESP Cell: ({target_cell_r_from_esp},{target_cell_c_from_esp})")

    if not connection_ok:
        if current_time - last_connection_attempt_time > 5.0: 
            setup_client_connection()
        left_motor.setVelocity(0.0); right_motor.setVelocity(0.0) 
        if webots_loop_counter % 10 == 1 : update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, webots_side_world_grid)
        continue
        
    # --- Obstacle Detection ---
    detected_obstacle_in_front_cell_rc = None
    if proximity_sensors:
        front_sensor_active = False
        max_prox_value = 0
        for sensor in proximity_sensors: 
            val = sensor.getValue()
            if val > max_prox_value: max_prox_value = val 
            if val > PROXIMITY_THRESHOLD:
                front_sensor_active = True
                break
        if front_sensor_active:
            obstacle_check_x = robot_pose['x'] + OBSTACLE_PROJECTION_DISTANCE * math.cos(robot_pose['theta'])
            obstacle_check_z = robot_pose['y'] + OBSTACLE_PROJECTION_DISTANCE * math.sin(robot_pose['theta'])
            obs_r, obs_c = world_to_grid_cell(obstacle_check_x, obstacle_check_z)
            if 0 <= obs_r < GRID_ROWS and 0 <= obs_c < GRID_COLS : 
                if webots_side_world_grid[obs_r][obs_c] == 0: 
                    if (obs_r, obs_c) not in last_obstacle_report_time or \
                       current_time - last_obstacle_report_time.get((obs_r, obs_c), 0) > OBSTACLE_REPORT_COOLDOWN:
                        detected_obstacle_in_front_cell_rc = (obs_r, obs_c)
                        print(f"Webots: Detected NEW obstacle at grid cell ({obs_r},{obs_c}). MaxProx: {max_prox_value:.0f}")
                        last_obstacle_report_time[(obs_r, obs_c)] = current_time
                        webots_side_world_grid[obs_r][obs_c] = 1
                        if fig_map and hasattr(update_live_map_plot_grid, 'initial_obstacles_drawn') and update_live_map_plot_grid.initial_obstacles_drawn: # type: ignore
                            plt.close(fig_map) 
                            fig_map, ax_map = None, None 
                            update_live_map_plot_grid.legend_added_flag = False # type: ignore
                            update_live_map_plot_grid.bounds_set_flag = False # type: ignore    
                            update_live_map_plot_grid.initial_obstacles_drawn = False # type: ignore
                            print("Webots: Plot re-initialized due to new obstacle.")

    # --- Prepare Message to ESP32 ---
    gs_data_str = f"GS:{s_left_on_line},{s_middle_on_line},{s_right_on_line}"
    pos_data_str = f"POS:{current_r_mapped},{current_c_mapped},{WEBOTS_TARGET_GOAL_ROW},{WEBOTS_TARGET_GOAL_COL}"
    message_to_esp32 = f"{gs_data_str};{pos_data_str}\n"

    if detected_obstacle_in_front_cell_rc:
        obs_msg = f"OBSTACLE:{detected_obstacle_in_front_cell_rc[0]},{detected_obstacle_in_front_cell_rc[1]}\n"
        message_to_esp32 = obs_msg + message_to_esp32 

    if webots_loop_counter % 20 == 1 or detected_obstacle_in_front_cell_rc:
        print(f"Webots TX: {message_to_esp32.strip().replace(chr(10), ' | ')}")


    try:
        client_sock.sendall(message_to_esp32.encode('utf-8'))
    except Exception as e:
        print(f"❌ Webots: Send Error: {e}. Disconnecting.")
        connection_ok = False
        if webots_loop_counter % 10 == 1 : update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, webots_side_world_grid)
        continue
    
    # --- Receive and Process Commands from ESP32 ---
    use_esp_direct_speeds = False # Reset flag each cycle
    try:
        data_bytes = client_sock.recv(256) 
        if not data_bytes:
            print("⚠️ Webots: ESP32 closed connection (recv empty).")
            connection_ok = False
        else:
            full_response_str = data_bytes.decode('utf-8')
            messages = full_response_str.splitlines()
            for msg_part in messages:
                msg = msg_part.strip()
                if not msg: continue
                
                if msg.startswith("PATH_GRID:"):
                    try:
                        path_str_payload = msg.split(":", 1)[1]
                        current_planned_path_nodes_grid = [tuple(map(int, cell.split(','))) for cell in path_str_payload.split(';')]
                    except Exception as ep:
                        print(f"Webots Warn: Parse PATH_GRID fail: '{msg}', E:{ep}")
                        current_planned_path_nodes_grid = []
                elif msg.startswith("GOTO:"): 
                    try:
                        coords_str = msg.split(":", 1)[1].split(',')
                        new_target_r, new_target_c = int(coords_str[0]), int(coords_str[1])
                        if new_target_r != target_cell_r_from_esp or new_target_c != target_cell_c_from_esp or not robot_is_moving_to_target_cell:
                             print(f"Webots RX GOTO: New target ({new_target_r},{new_target_c}). Old/Current: ({target_cell_r_from_esp},{target_cell_c_from_esp}), WasMoving: {robot_is_moving_to_target_cell}")
                        target_cell_r_from_esp, target_cell_c_from_esp = new_target_r, new_target_c
                        robot_is_moving_to_target_cell = True
                        use_esp_direct_speeds = False 
                    except Exception as eg:
                        print(f"Webots Warn: Parse GOTO fail: '{msg}', E:{eg}")
                elif msg.startswith("SPEEDS:"): 
                    try:
                        speeds_str = msg.split(":", 1)[1].split(',')
                        if len(speeds_str) == 2:
                            esp_commanded_left_speed = float(speeds_str[0])
                            esp_commanded_right_speed = float(speeds_str[1])
                            use_esp_direct_speeds = True
                            robot_is_moving_to_target_cell = False 
                            if webots_loop_counter % 5 == 1 or esp_commanded_left_speed != 0 or esp_commanded_right_speed != 0: # Print if moving or periodically
                                print(f"Webots RX SPEEDS: L={esp_commanded_left_speed:.2f}, R={esp_commanded_right_speed:.2f}")
                        else:
                            print(f"Webots Warn: Parse SPEEDS fail, wrong parts: '{msg}'")
                    except Exception as es:
                        print(f"Webots Warn: Parse SPEEDS fail: '{msg}', E:{es}")
                elif msg == "stop":
                    print(f"Webots RX: STOP command from ESP.")
                    robot_is_moving_to_target_cell = False
                    use_esp_direct_speeds = True 
                    esp_commanded_left_speed = 0.0
                    esp_commanded_right_speed = 0.0
                    target_cell_r_from_esp, target_cell_c_from_esp = -1, -1 
                    break 
                else: 
                    print(f"⚠️ Webots: Unhandled msg from ESP32: '{msg}' (repr: {repr(msg)})") # Added repr for debugging
    except socket.timeout: 
        if webots_loop_counter % 100 == 1: print("Webots: Timeout receiving from ESP32. Continuing with last command/state.")
        pass 
    except Exception as e:
        print(f"❌ Webots: Recv Error: {e}. Disconnecting.")
        connection_ok = False
    
    if not connection_ok:
        if webots_loop_counter % 10 == 1 : update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, webots_side_world_grid)
        continue

    # --- Execute Movement Command ---
    left_speed_cmd, right_speed_cmd = 0.0, 0.0

    if use_esp_direct_speeds:
        left_speed_cmd = esp_commanded_left_speed
        right_speed_cmd = esp_commanded_right_speed
        # Optional: Print direct speeds if they are non-zero or periodically
        # if webots_loop_counter % 10 == 1 and (left_speed_cmd != 0 or right_speed_cmd != 0):
        #      print(f"Debug DirectSpeeds Applied: L={left_speed_cmd:.2f}, R={right_speed_cmd:.2f}")
    elif robot_is_moving_to_target_cell and target_cell_r_from_esp != -1: 
        target_world_x, target_world_z = grid_cell_to_world_center(target_cell_r_from_esp, target_cell_c_from_esp)
        delta_x_to_target = target_world_x - robot_pose['x']
        delta_z_to_target = target_world_z - robot_pose['y']
        distance_to_target_center = math.sqrt(delta_x_to_target**2 + delta_z_to_target**2)
        angle_to_target_center = math.atan2(delta_z_to_target, delta_x_to_target)
        angle_difference = math.atan2(math.sin(angle_to_target_center - robot_pose['theta']), 
                                      math.cos(angle_to_target_center - robot_pose['theta']))
        
        if webots_loop_counter % 20 == 2: 
            print(f"Debug Motion (Grid): To ({target_cell_r_from_esp},{target_cell_c_from_esp}). Dist: {distance_to_target_center:.3f}, AngleDiff: {math.degrees(angle_difference):.1f} deg. MappedCell: ({current_r_mapped},{current_c_mapped})")

        is_at_target_cell_center_dist = distance_to_target_center <= DIST_TO_CELL_THRESHOLD
        is_in_correct_mapped_cell = (current_r_mapped, current_c_mapped) == (target_cell_r_from_esp, target_cell_c_from_esp)

        if is_at_target_cell_center_dist and is_in_correct_mapped_cell:
            print(f"Webots: Correctly ARRIVED at cell ({target_cell_r_from_esp},{target_cell_c_from_esp}). Robot grid pos: ({current_r_mapped},{current_c_mapped})")
            robot_is_moving_to_target_cell = False 
            left_speed_cmd, right_speed_cmd = 0.0, 0.0
        elif is_at_target_cell_center_dist and not is_in_correct_mapped_cell:
            print(f"Webots: Near target ({target_cell_r_from_esp},{target_cell_c_from_esp}) center, BUT mapped to ({current_r_mapped},{current_c_mapped}). Stopping. ESP32 should replan.")
            robot_is_moving_to_target_cell = False 
            left_speed_cmd, right_speed_cmd = 0.0, 0.0
        else: 
            if abs(angle_difference) > ANGLE_TO_CELL_THRESHOLD: 
                turn_effort_gain = 25.0 
                turn_scaling_factor = angle_difference * turn_effort_gain 
                turn_speed_val = max(-TURN_SPEED_CELL_NAV, min(TURN_SPEED_CELL_NAV, turn_scaling_factor * TURN_SPEED_CELL_NAV))
                
                left_speed_cmd = -turn_speed_val
                right_speed_cmd = turn_speed_val
                if webots_loop_counter % 20 == 3:
                     print(f"Debug Turn CMD (Grid): L:{left_speed_cmd:.2f} R:{right_speed_cmd:.2f} (ScaleFactor:{turn_scaling_factor:.2f}, Val:{turn_speed_val:.2f})")
            else: 
                left_speed_cmd = MAX_SPEED_CELL_NAV
                right_speed_cmd = MAX_SPEED_CELL_NAV
            
    left_motor.setVelocity(left_speed_cmd)
    right_motor.setVelocity(right_speed_cmd)

    if encoders_ok and webots_loop_counter % 5 == 1: 
        update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, webots_side_world_grid)

# --- End of Simulation ---
if client_sock:
    try: client_sock.sendall("stop\n".encode('utf-8')) 
    except: pass
    client_sock.close()
    print("ℹ️ Webots: Client socket closed.")
if fig_map: 
    print("ℹ️ Webots: Simulation ended. Plot window active. Close plot to exit.")
    plt.ioff()
    plt.show(block=True)
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
print("ℹ️ Webots: Controller finished.")

