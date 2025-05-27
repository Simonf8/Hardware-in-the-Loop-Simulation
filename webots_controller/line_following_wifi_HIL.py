# line_following_wifi_HIL.py (Webots Controller - Grid Navigation Client - FIXED)

from controller import Robot
import socket
import time
import math
import matplotlib.pyplot as plt

# --- Network Configuration ---
ESP32_IP_ADDRESS = "192.168.x.x"  # <<<<<<<<<<< SET ESP32's ACTUAL IP
ESP32_PORT = 8266

# --- Robot Parameters ---
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052

# --- Grid Map Configuration ---
GRID_ROWS = 7
GRID_COLS = 9
GRID_CELL_SIZE = 0.1
GRID_ORIGIN_X = 0.0
GRID_ORIGIN_Z = 0.6

WEBOTS_TARGET_GOAL_ROW = 6
WEBOTS_TARGET_GOAL_COL = 8

MAX_SPEED_CELL_NAV = 0.5
TURN_SPEED_CELL_NAV = 0.5
DIST_TO_CELL_THRESHOLD = GRID_CELL_SIZE * 0.15
ANGLE_TO_CELL_THRESHOLD = 0.1  # radians

# --- Map Helper Functions ---
def grid_cell_to_world_center(r, c):
    """Given grid cell row,col, return (world_x, world_z) of cell center."""
    wx = GRID_ORIGIN_X + c * GRID_CELL_SIZE
    wz = GRID_ORIGIN_Z - r * GRID_CELL_SIZE
    return wx, wz

def world_to_grid_cell(x, z):
    """Given world (x, z), return (row, col) indices of closest grid cell."""
    c = int(round((x - GRID_ORIGIN_X) / GRID_CELL_SIZE))
    r = int(round((GRID_ORIGIN_Z - z) / GRID_CELL_SIZE))
    c = max(0, min(GRID_COLS - 1, c))
    r = max(0, min(GRID_ROWS - 1, r))
    return r, c

# --- Plotting function (as before) ---
plt.ion()
fig_map, ax_map = None, None
robot_path_trail_plot = []
current_planned_path_nodes_grid = []
plot_dynamic_artists = []
webots_loop_counter = 0

def update_live_map_plot_grid(current_odom_pose, current_grid_cell_tuple, planned_grid_path=None, obstacles_grid_data=None):
    global fig_map, ax_map, robot_path_trail_plot, plot_dynamic_artists, webots_loop_counter
    if fig_map is None:
        fig_map, ax_map = plt.subplots(figsize=(8, 8))
        ax_map.set_aspect('equal', 'box')
        ax_map.set_xlabel("World X (m)")
        ax_map.set_ylabel("World Z (m)")
        ax_map.set_title("Robot Grid Navigation")
        world_grid_lines_x = [GRID_ORIGIN_X + (c - 0.5) * GRID_CELL_SIZE for c in range(GRID_COLS + 1)]
        world_grid_lines_z = [GRID_ORIGIN_Z - (r - 0.5) * GRID_CELL_SIZE for r in range(GRID_ROWS + 1)]
        for wx in world_grid_lines_x:
            ax_map.plot([wx, wx], [min(world_grid_lines_z), max(world_grid_lines_z)], 'k-', alpha=0.2, lw=0.5)
        for wz in world_grid_lines_z:
            ax_map.plot([min(world_grid_lines_x), max(world_grid_lines_x)], [wz, wz], 'k-', alpha=0.2, lw=0.5)
        ax_map.invert_yaxis()
    for artist in plot_dynamic_artists:
        artist.remove()
    plot_dynamic_artists = []
    if obstacles_grid_data:
        for r_idx in range(len(obstacles_grid_data)):
            for c_idx in range(len(obstacles_grid_data[0])):
                if obstacles_grid_data[r_idx][c_idx] == 1:
                    wx, wz = grid_cell_to_world_center(r_idx, c_idx)
                    rect = plt.Rectangle((wx - GRID_CELL_SIZE / 2, wz - GRID_CELL_SIZE / 2),
                                        GRID_CELL_SIZE, GRID_CELL_SIZE, fc='dimgray', alpha=0.7, zorder=0)
                    ax_map.add_patch(rect)
                    plot_dynamic_artists.append(rect)
    robot_path_trail_plot.append((current_odom_pose['x'], current_odom_pose['y']))
    if len(robot_path_trail_plot) > 300:
        robot_path_trail_plot.pop(0)
    if len(robot_path_trail_plot) > 1:
        line, = ax_map.plot(*zip(*robot_path_trail_plot), 'c-', lw=1.5, label='Odometry Trail', zorder=4)
        plot_dynamic_artists.append(line)
    if planned_grid_path and len(planned_grid_path) > 1:
        path_world_x = []
        path_world_z = []
        for r_node, c_node in planned_grid_path:
            wx, wz = grid_cell_to_world_center(r_node, c_node)
            path_world_x.append(wx)
            path_world_z.append(wz)
        line, = ax_map.plot(path_world_x, path_world_z, 'b*--', lw=2, label='Planned Path (ESP32)', zorder=5, ms=6)
        plot_dynamic_artists.append(line)
    rx, ry_is_z, rth = current_odom_pose['x'], current_odom_pose['y'], current_odom_pose['theta']
    dot, = ax_map.plot(rx, ry_is_z, 'ro', ms=8, label='Current Position', zorder=6)
    plot_dynamic_artists.append(dot)
    arrow_len = GRID_CELL_SIZE * 0.4
    arrow = ax_map.arrow(rx, ry_is_z, arrow_len * math.cos(rth), arrow_len * math.sin(rth),
                         head_width=arrow_len * 0.4, head_length=arrow_len * 0.6,
                         fc='maroon', ec='maroon', zorder=7, lw=1.5)
    plot_dynamic_artists.append(arrow)
    if current_grid_cell_tuple:
        curr_cell_wx, curr_cell_wz = grid_cell_to_world_center(current_grid_cell_tuple[0], current_grid_cell_tuple[1])
        rect_curr = plt.Rectangle((curr_cell_wx - GRID_CELL_SIZE / 2, curr_cell_wz - GRID_CELL_SIZE / 2),
                                 GRID_CELL_SIZE, GRID_CELL_SIZE, facecolor='yellow', alpha=0.3, zorder=0)
        ax_map.add_patch(rect_curr)
        plot_dynamic_artists.append(rect_curr)
    if not hasattr(update_live_map_plot_grid, 'legend_added'):
        ax_map.legend(loc='upper right', fontsize='small')
        update_live_map_plot_grid.legend_added = True
    if not hasattr(update_live_map_plot_grid, 'bounds_set') or webots_loop_counter % 200 == 1:
        min_wx = GRID_ORIGIN_X - GRID_CELL_SIZE
        max_wx = GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE
        min_wz = GRID_ORIGIN_Z - GRID_ROWS * GRID_CELL_SIZE
        max_wz = GRID_ORIGIN_Z + GRID_CELL_SIZE
        ax_map.set_xlim(min_wx, max_wx)
        ax_map.set_ylim(max_wz, min_wz)
        update_live_map_plot_grid.bounds_set = True
    plt.draw()
    plt.pause(0.001)

# --- Initialize Robot & Devices ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # 'y' stores world Z
prev_left_enc = 0.0
prev_right_enc = 0.0
first_odom = True

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
if not (left_motor and right_motor):
    print("Motors missing")
    exit(1)
for m in (left_motor, right_motor):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
print("✅ Webots: Motors initialized.")

left_enc = robot.getDevice('left wheel sensor')
right_enc = robot.getDevice('right wheel sensor')
encoders_ok = False
if left_enc and right_enc:
    left_enc.enable(timestep)
    right_enc.enable(timestep)
    encoders_ok = True
    print("✅ Webots: Encoders enabled.")
else:
    print("⚠️ Webots: Encoders missing.")

# --- World grid: Optionally define obstacles here as 2D list (0=open, 1=obstacle) ---
world_grid = [[0]*GRID_COLS for _ in range(GRID_ROWS)]

# --- Network Client ---
client_sock = None
conn_ok = False
last_conn_attempt = 0

def setup_client():
    global client_sock, conn_ok, last_conn_attempt
    if client_sock:
        try:
            client_sock.close()
        except:
            pass
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_sock.settimeout(3.0)
    print(f"Webots: Connect ESP32 @ {ESP32_IP_ADDRESS}:{ESP32_PORT}...")
    try:
        client_sock.connect((ESP32_IP_ADDRESS, ESP32_PORT))
        client_sock.settimeout(1.0)
        conn_ok = True
        print("✅ Webots: Connected.")
    except Exception as e:
        print(f"⚠️ Webots: Connect fail: {e}")
        conn_ok = False
    last_conn_attempt = time.time()
    return conn_ok

setup_client()

# --- Initial pose for plotting (assume start at (0,0) facing East) ---
initial_world_x, initial_world_z = grid_cell_to_world_center(0, 0)
robot_pose['x'] = initial_world_x
robot_pose['y'] = initial_world_z
robot_pose['theta'] = 0.0  # facing +X

first_odom = True  # re-init odometry baseline

target_cell_r, target_cell_c = -1, -1
robot_is_moving_to_target = False

print(f"\nWebots: Controller for Grid Navigation started.")

while robot.step(timestep) != -1:
    webots_loop_counter += 1
    # 1. Odometry
    if encoders_ok:
        curr_L = left_enc.getValue()
        curr_R = right_enc.getValue()
        if first_odom:
            prev_left_enc = curr_L
            prev_right_enc = curr_R
            first_odom = False
        else:
            dL = (curr_L - prev_left_enc) * WHEEL_RADIUS
            dR = (curr_R - prev_right_enc) * WHEEL_RADIUS
            dC = (dL + dR) / 2.0
            dTh = (dR - dL) / AXLE_LENGTH
            robot_pose['x'] += dC * math.cos(robot_pose['theta'] + dTh / 2.0)
            robot_pose['y'] += dC * math.sin(robot_pose['theta'] + dTh / 2.0)
            robot_pose['theta'] += dTh
            robot_pose['theta'] = math.atan2(math.sin(robot_pose['theta']), math.cos(robot_pose['theta']))
            prev_left_enc = curr_L
            prev_right_enc = curr_R

    # 2. Current grid cell
    current_r_mapped, current_c_mapped = world_to_grid_cell(robot_pose['x'], robot_pose['y'])

    # 3. Network & comms
    if not conn_ok:
        if time.time() - last_conn_attempt > 5.0:
            setup_client()
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        time.sleep(0.1)
        continue

    # Send position to ESP32 (always 4 fields)
    pos_msg = f"POS:{current_r_mapped},{current_c_mapped},{WEBOTS_TARGET_GOAL_ROW},{WEBOTS_TARGET_GOAL_COL}\n"
    try:
        client_sock.sendall(pos_msg.encode('utf-8'))
    except Exception as e:
        print(f"❌ Webots Send POS Error: {e}")
        conn_ok = False
        continue

    # Receive command from ESP32
    try:
        client_sock.settimeout(0.05)
        data_bytes = client_sock.recv(256)
        client_sock.settimeout(1.0)
        if not data_bytes:
            print("⚠️ Webots: ESP32 closed connection.")
            conn_ok = False
        else:
            full_resp = data_bytes.decode('utf-8')
            msgs = full_resp.splitlines()
            for msg_part in msgs:
                msg = msg_part.strip()
                if not msg:
                    continue
                if msg.startswith("PATH_GRID:"):
                    try:
                        p_str = msg.split(":", 1)[1]
                        current_planned_path_nodes_grid = [
                            tuple(map(int, cell.split(',')))
                            for cell in p_str.split(';') if cell
                        ]
                    except Exception as ep:
                        print(f"Webots Warn: Parse PATH_GRID fail:'{msg}',E:{ep}")
                        current_planned_path_nodes_grid = []
                elif msg.startswith("GOTO:"):
                    try:
                        coords_str = msg.split(":", 1)[1].split(',')
                        target_cell_r, target_cell_c = int(coords_str[0]), int(coords_str[1])
                        robot_is_moving_to_target = True
                    except Exception as e_goto:
                        print(f"Webots Warn: Parse GOTO fail:'{msg}',E:{e_goto}")
                        robot_is_moving_to_target = False
                elif msg == "stop":
                    robot_is_moving_to_target = False
                    target_cell_r, target_cell_c = -1, -1
                    print("Webots Rcvd STOP from ESP32")
                elif msg:
                    print(f"⚠️ Webots: Unhandled msg from ESP32: '{msg}'")
        if not conn_ok:
            continue
    except socket.timeout:
        pass
    except Exception as e:
        print(f"❌ Webots Recv Error: {e}")
        conn_ok = False
    if not conn_ok:
        continue

    # 4. Movement logic
    left_speed, right_speed = 0.0, 0.0
    if robot_is_moving_to_target and target_cell_r != -1:
        target_wx, target_wz = grid_cell_to_world_center(target_cell_r, target_cell_c)
        dx = target_wx - robot_pose['x']
        dz = target_wz - robot_pose['y']
        distance_to_target = math.sqrt(dx * dx + dz * dz)
        angle_to_target = math.atan2(dz, dx)
        angle_diff = math.atan2(math.sin(angle_to_target - robot_pose['theta']),
                                math.cos(angle_to_target - robot_pose['theta']))

        if distance_to_target > DIST_TO_CELL_THRESHOLD:
            if abs(angle_diff) > ANGLE_TO_CELL_THRESHOLD:
                turn_val = max(-TURN_SPEED_CELL_NAV, min(TURN_SPEED_CELL_NAV, angle_diff * 2.0))
                left_speed = -turn_val
                right_speed = turn_val
            else:
                left_speed = MAX_SPEED_CELL_NAV
                right_speed = MAX_SPEED_CELL_NAV
        else:
            robot_is_moving_to_target = False
            left_speed, right_speed = 0.0, 0.0

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # Plot
    if encoders_ok and webots_loop_counter % 5 == 1:
        update_live_map_plot_grid(robot_pose, (current_r_mapped, current_c_mapped), current_planned_path_nodes_grid, world_grid)

# Cleanup
if client_sock:
    client_sock.close()
    print("ℹ️ Webots: Client socket closed.")
if fig_map:
    plt.ioff()
    plt.show(block=True)
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
print("ℹ️ Webots: Controller finished.")
