"""Clean Webots HIL Controller with Live Dashboard and ESP32 Dijkstra Path Planning (Enhanced Turns)"""
from controller import Robot
import socket
import time
import math
import matplotlib.pyplot as plt
# import numpy as np # Not strictly needed in this version after refactor
import json

# --- Network Configuration ---
ESP32_IP_ADDRESS = "192.168.53.193"  # UPDATE WITH YOUR ESP32 IP (from your image)
ESP32_PORT = 8080

# --- Robot Parameters ---
WHEEL_RADIUS = 0.0205  # meters
AXLE_LENGTH = 0.052    # meters (distance between wheel centers)

# --- Grid Configuration (Must match ESP32) ---
GRID_ROWS = 15
GRID_COLS = 17
GRID_CELL_SIZE = 0.06  # meters
GRID_ORIGIN_X = -0.40
GRID_ORIGIN_Z = -0.30
GOAL_ROW = 14
GOAL_COL = 0

# --- Control Parameters ---
FORWARD_SPEED = 2.8    # rad/s
TURN_SPEED_FACTOR = 0.7
LINE_THRESHOLD = 600
LOST_LINE_TURN_SPEED = 1.5

# --- Enhanced Turn Parameters ---
MIN_INITIAL_SPIN_DURATION = 0.35  # seconds (Tune: time for initial spin)
MAX_SEARCH_SPIN_DURATION = 2.5    # seconds (Tune: max time to search for line during turn)
MAX_ADJUST_DURATION = 1.5         # seconds (Tune: max time to adjust on a found line)
TURN_ADJUST_SPEED_FACTOR = 0.4    # Speed factor for adjusting on line after turn

# --- Grid map (same as ESP32) ---
world_grid = [
    [1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],  # Row 0
    [1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],  # Row 1
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 2
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],  # Row 3
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],  # Row 4
    [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0],  # Row 5
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],  # Row 6
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 7
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],  # Row 8
    [0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0],  # Row 9
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],  # Row 10
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],  # Row 11
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],  # Row 12
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1],  # Row 13
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1]   # Row 14
]

# --- Visualization variables ---
plt.ion()
fig = None
ax = None
robot_trail_world = []
planned_path_grid = []

# --- Webots Internal State for Turns ---
webots_internal_turn_phase = 'NONE' # NONE, INITIATE_SPIN, SEARCHING_LINE, ADJUSTING_ON_LINE
webots_turn_command_active = None   # Stores the ESP32 turn command ('turn_left' or 'turn_right')
turn_phase_start_time = 0.0

def world_to_grid(world_x, world_z):
    col = int(round((world_x - GRID_ORIGIN_X) / GRID_CELL_SIZE - 0.5))
    row = int(round((world_z - GRID_ORIGIN_Z) / GRID_CELL_SIZE - 0.5))
    return max(0, min(row, GRID_ROWS - 1)), max(0, min(col, GRID_COLS - 1))

def grid_to_world_center(row, col):
    world_x = GRID_ORIGIN_X + (col + 0.5) * GRID_CELL_SIZE
    world_z = GRID_ORIGIN_Z + (row + 0.5) * GRID_CELL_SIZE
    return world_x, world_z

def update_visualization(robot_world_pose, current_robot_grid_pos, path_from_esp32):
    global fig, ax, robot_trail_world, planned_path_grid
    if fig is None:
        fig, ax = plt.subplots(figsize=(14, 10)); ax.set_aspect('equal')
        ax.set_title('🤖 HIL Robot Dijkstra Path Planning 🎯 (Webots View)', fontsize=16, fontweight='bold')
        ax.set_xlabel('World X (m)'); ax.set_ylabel('World Z (m)')
        for r_idx in range(GRID_ROWS + 1):
            z_coord = GRID_ORIGIN_Z + r_idx * GRID_CELL_SIZE
            ax.plot([GRID_ORIGIN_X, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE], [z_coord, z_coord], 'k-', alpha=0.2, lw=0.5)
        for c_idx in range(GRID_COLS + 1):
            x_coord = GRID_ORIGIN_X + c_idx * GRID_CELL_SIZE
            ax.plot([x_coord, x_coord], [GRID_ORIGIN_Z, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE], 'k-', alpha=0.2, lw=0.5)
        for r_idx in range(GRID_ROWS):
            for c_idx in range(GRID_COLS):
                wx_cell, wz_cell = grid_to_world_center(r_idx, c_idx)
                cell_color = 'black' if world_grid[r_idx][c_idx] == 0 else 'lightgrey'
                alpha_val = 0.6 if world_grid[r_idx][c_idx] == 0 else 0.3
                rect = plt.Rectangle((wx_cell - GRID_CELL_SIZE / 2, wz_cell - GRID_CELL_SIZE / 2), GRID_CELL_SIZE, GRID_CELL_SIZE, facecolor=cell_color, alpha=alpha_val, edgecolor='gray', lw=0.5)
                ax.add_patch(rect)
                if r_idx % 3 == 0 and c_idx % 3 == 0: ax.text(wx_cell, wz_cell, f'({r_idx},{c_idx})', ha='center', va='center', fontsize=7, color='blue', alpha=0.5)
        margin = GRID_CELL_SIZE * 1.5
        ax.set_xlim(GRID_ORIGIN_X - margin, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE + margin)
        ax.set_ylim(GRID_ORIGIN_Z - margin, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE + margin)
        from matplotlib.patches import Patch
        legend_elements = [Patch(facecolor='black', alpha=0.6, label='Pathable'), Patch(facecolor='lightgrey', alpha=0.3, label='Obstacle'),
                           plt.Line2D([0], [0], color='cyan', lw=2, label='Robot Trail'), plt.Line2D([0], [0], color='magenta', marker='o',ms=5, ls='--', lw=2, label='Dijkstra Path'),
                           plt.Line2D([0], [0], color='red', marker='o',ms=8, ls='', label='Robot'), plt.Line2D([0], [0], color='green', marker='*',ms=12, ls='', label='Goal')]
        ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1)); plt.tight_layout(rect=[0, 0, 0.85, 1]); plt.show(block=False); plt.pause(0.01)

    for artist_list in [ax.lines[ (GRID_ROWS + 1) + (GRID_COLS + 1):], ax.patches[GRID_ROWS * GRID_COLS:], ax.texts[ (GRID_ROWS//3 * GRID_COLS//3 * (GRID_COLS//3 >0)):]]:
        while artist_list: artist_list.pop(0).remove()

    robot_trail_world.append((robot_world_pose['x'], robot_world_pose['z']))
    if len(robot_trail_world) > 150: robot_trail_world.pop(0)
    if len(robot_trail_world) > 1: trail_x, trail_z = zip(*robot_trail_world); ax.plot(trail_x, trail_z, 'cyan', lw=2, alpha=0.7)

    if path_from_esp32 and len(path_from_esp32) > 1:
        planned_path_grid = path_from_esp32
        path_world_coords = [grid_to_world_center(r, c) for r, c in planned_path_grid]
        path_x, path_z = zip(*path_world_coords)
        ax.plot(path_x, path_z, 'mo--', lw=2, ms=5, alpha=0.8)
        if path_x: ax.plot(path_x[0], path_z[0], 'm^', ms=8); ax.plot(path_x[-1], path_z[-1], 'm*', ms=8)

    ax.plot(robot_world_pose['x'], robot_world_pose['z'], 'ro', ms=10, mec='darkred', mew=1)
    arrow_len = GRID_CELL_SIZE * 0.8; dx = arrow_len * math.cos(robot_world_pose['theta']); dz = arrow_len * math.sin(robot_world_pose['theta'])
    from matplotlib.patches import FancyArrowPatch
    arrow = FancyArrowPatch((robot_world_pose['x'], robot_world_pose['z']), (robot_world_pose['x'] + dx, robot_world_pose['z'] + dz), arrowstyle='->', mutation_scale=15, color='darkred', lw=2)
    ax.add_patch(arrow)
    if current_robot_grid_pos:
        wx_curr, wz_curr = grid_to_world_center(current_robot_grid_pos[0], current_robot_grid_pos[1])
        rect_curr = plt.Rectangle((wx_curr - GRID_CELL_SIZE/2, wz_curr - GRID_CELL_SIZE/2), GRID_CELL_SIZE, GRID_CELL_SIZE, edgecolor='yellow', facecolor='yellow', alpha=0.3, lw=2)
        ax.add_patch(rect_curr)
    goal_wx, goal_wz = grid_to_world_center(GOAL_ROW, GOAL_COL); ax.plot(goal_wx, goal_wz, 'g*', ms=15, mec='darkgreen', mew=1.5)
    info_text_content = (f"Grid: {current_robot_grid_pos} -> {GOAL_ROW, GOAL_COL}\n"
                         f"World: X={robot_world_pose['x']:.2f}, Z={robot_world_pose['z']:.2f}, θ={math.degrees(robot_world_pose['theta']):.1f}°\n"
                         f"Path Nodes: {len(planned_path_grid) if planned_path_grid else 'N/A'}\n"
                         f"Webots Turn Phase: {webots_internal_turn_phase}")
    if hasattr(ax, '_info_text_handle') and ax._info_text_handle in ax.texts: ax._info_text_handle.remove()
    ax._info_text_handle = ax.text(0.02, 0.98, info_text_content, transform=ax.transAxes, va='top', fontsize=8, bbox=dict(boxstyle='round,pad=0.3', fc='lightblue', alpha=0.7))
    plt.draw(); plt.pause(0.001)

robot = Robot(); timestep = int(robot.getBasicTimeStep())
robot_world_pose = {'x': 0.0, 'z': 0.0, 'theta': 0.0}; prev_left_enc = 0.0; prev_right_enc = 0.0; first_run_odometry = True
left_motor = robot.getDevice('left wheel motor'); right_motor = robot.getDevice('right wheel motor')
left_enc = robot.getDevice('left wheel sensor'); right_enc = robot.getDevice('right wheel sensor')
for motor in [left_motor, right_motor]: motor.setPosition(float('inf')); motor.setVelocity(0.0)
for encoder in [left_enc, right_enc]: encoder.enable(timestep)
ground_sensors_wb = [];传感器名称列表 = ['gs0', 'gs1', 'gs2']
for name in 传感器名称列表: sensor = robot.getDevice(name); sensor.enable(timestep); ground_sensors_wb.append(sensor)

client_sock = None; connection_ok = False; esp32_current_command = 'stop'

def connect_to_esp32():
    global client_sock, connection_ok
    try:
        if client_sock: client_sock.close()
        client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM); client_sock.settimeout(2.0)
        client_sock.connect((ESP32_IP_ADDRESS, ESP32_PORT)); client_sock.settimeout(0.05)
        connection_ok = True; print(f"✅ Connected to ESP32 at {ESP32_IP_ADDRESS}:{ESP32_PORT}"); return True
    except Exception as e:
        print(f"🔌 ESP32 Connection failed: {e}"); connection_ok = False
        if client_sock: client_sock.close(); client_sock = None; return False

INITIAL_GRID_ROW, INITIAL_GRID_COL = 2, 16
robot_world_pose['x'], robot_world_pose['z'] = grid_to_world_center(INITIAL_GRID_ROW, INITIAL_GRID_COL)
robot_world_pose['theta'] = math.pi / 2.0
current_robot_grid_pos = world_to_grid(robot_world_pose['x'], robot_world_pose['z'])
print(f"🚀 Robot init @ grid {current_robot_grid_pos}, World X={robot_world_pose['x']:.2f}, Z={robot_world_pose['z']:.2f}, Goal {GOAL_ROW,GOAL_COL}")

loop_counter = 0; last_connection_attempt_time = 0; last_data_send_time = 0

while robot.step(timestep) != -1:
    if loop_counter == 0: connect_to_esp32(); update_visualization(robot_world_pose, current_robot_grid_pos, planned_path_grid)
    loop_counter += 1; current_time = robot.getTime()

    if not first_run_odometry:
        left_enc_val, right_enc_val = left_enc.getValue(), right_enc.getValue()
        delta_dist = ((left_enc_val - prev_left_enc) * WHEEL_RADIUS + (right_enc_val - prev_right_enc) * WHEEL_RADIUS) / 2.0
        delta_theta = ((right_enc_val - prev_right_enc) * WHEEL_RADIUS - (left_enc_val - prev_left_enc) * WHEEL_RADIUS) / AXLE_LENGTH
        robot_world_pose['x'] += delta_dist * math.cos(robot_world_pose['theta'] + delta_theta / 2.0)
        robot_world_pose['z'] += delta_dist * math.sin(robot_world_pose['theta'] + delta_theta / 2.0)
        robot_world_pose['theta'] = math.atan2(math.sin(robot_world_pose['theta'] + delta_theta), math.cos(robot_world_pose['theta'] + delta_theta))
        prev_left_enc, prev_right_enc = left_enc_val, right_enc_val
    else: prev_left_enc, prev_right_enc = left_enc.getValue(), right_enc.getValue(); first_run_odometry = False

    current_robot_grid_pos = world_to_grid(robot_world_pose['x'], robot_world_pose['z'])
    raw_sensor_values = [gs.getValue() for gs in ground_sensors_wb]
    line_detected_flags = [1 if val < LINE_THRESHOLD else 0 for val in raw_sensor_values]
    on_left_sensor, on_center_sensor, on_right_sensor = line_detected_flags

    if not connection_ok:
        if current_time - last_connection_attempt_time > 3.0: connect_to_esp32(); last_connection_attempt_time = current_time
        left_motor.setVelocity(0.0); right_motor.setVelocity(0.0)
        if loop_counter % 10 == 0: update_visualization(robot_world_pose, current_robot_grid_pos, planned_path_grid)
        continue

    if current_time - last_data_send_time > 0.1:
        try:
            data_to_esp = {'type': 'webots_status', 'robot_grid_pos': list(current_robot_grid_pos), 'goal_grid_pos': [GOAL_ROW, GOAL_COL],
                           'world_pose': {'x': round(robot_world_pose['x'],3), 'z': round(robot_world_pose['z'],3), 'theta_rad': round(robot_world_pose['theta'],3)},
                           'sensors_binary': line_detected_flags }
            client_sock.sendall((json.dumps(data_to_esp) + '\n').encode('utf-8')); last_data_send_time = current_time
        except Exception as e: print(f"❌ ESP Send: {e}"); connection_ok = False; client_sock.close(); client_sock=None; continue
    try:
        response_bytes = client_sock.recv(1024)
        if response_bytes:
            for msg_part in response_bytes.decode('utf-8').strip().split('\n'):
                if not msg_part.strip(): continue
                try:
                    data_from_esp = json.loads(msg_part)
                    if data_from_esp.get('type') == 'esp32_command':
                        new_esp_cmd = data_from_esp.get('action', 'stop')
                        if new_esp_cmd != esp32_current_command and esp32_current_command in ['turn_left', 'turn_right'] and new_esp_cmd not in ['turn_left', 'turn_right']:
                            print(f"Webots: ESP32 cmd '{new_esp_cmd}' overrides active turn. Resetting turn state.")
                            webots_internal_turn_phase = 'NONE' # ESP took over, stop local turning
                            webots_turn_command_active = None
                        esp32_current_command = new_esp_cmd
                        planned_path_grid = data_from_esp.get('path', planned_path_grid) # Keep old path if new one not sent
                except json.JSONDecodeError as e: print(f"⚠️ JSON ESP: '{msg_part}', {e}")
                except Exception as e: print(f"⚠️ Proc ESP: {e}")
    except socket.timeout: pass
    except Exception as e: print(f"❌ ESP Recv: {e}"); connection_ok = False; client_sock.close(); client_sock=None; continue

    left_speed, right_speed = 0.0, 0.0

    # --- Handle ESP32 Commands with Enhanced Turning ---
    if esp32_current_command not in ['turn_left', 'turn_right'] and webots_internal_turn_phase != 'NONE':
        # If ESP command is not a turn, but Webots was in a turn phase, reset Webots turn state.
        # This ensures 'forward' or 'stop' from ESP takes immediate precedence over local turning.
        # print(f"Webots: ESP cmd '{esp32_current_command}' received. Clearing internal turn phase '{webots_internal_turn_phase}'.")
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None

    if esp32_current_command == 'stop':
        left_speed, right_speed = 0.0, 0.0
        webots_internal_turn_phase = 'NONE'; webots_turn_command_active = None # Ensure reset
    elif esp32_current_command == 'forward':
        webots_internal_turn_phase = 'NONE'; webots_turn_command_active = None # Ensure reset
        if on_center_sensor and not on_left_sensor and not on_right_sensor: left_speed, right_speed = FORWARD_SPEED, FORWARD_SPEED
        elif on_left_sensor and not on_right_sensor: left_speed, right_speed = FORWARD_SPEED * 0.3, FORWARD_SPEED * 0.8
        elif on_right_sensor and not on_left_sensor: left_speed, right_speed = FORWARD_SPEED * 0.8, FORWARD_SPEED * 0.3
        elif on_left_sensor and on_center_sensor and not on_right_sensor: left_speed, right_speed = FORWARD_SPEED * 0.5, FORWARD_SPEED * 0.9
        elif on_right_sensor and on_center_sensor and not on_left_sensor: left_speed, right_speed = FORWARD_SPEED * 0.9, FORWARD_SPEED * 0.5
        elif not any(line_detected_flags): left_speed, right_speed = FORWARD_SPEED * 0.2, FORWARD_SPEED * 0.2 # Lost line, crawl
        else: left_speed, right_speed = FORWARD_SPEED * 0.4, FORWARD_SPEED * 0.4 # All sensors, intersection or broad line
    
    elif esp32_current_command in ['turn_left', 'turn_right']:
        if webots_turn_command_active != esp32_current_command or webots_internal_turn_phase == 'NONE': # New/Restart turn
            webots_turn_command_active = esp32_current_command
            webots_internal_turn_phase = 'INITIATE_SPIN'
            turn_phase_start_time = current_time
            print(f"Webots: Turn '{webots_turn_command_active}' state -> INITIATE_SPIN")

        if webots_internal_turn_phase == 'INITIATE_SPIN':
            spin_speed_inner = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.6
            spin_speed_outer = FORWARD_SPEED * TURN_SPEED_FACTOR * 1.0
            left_speed, right_speed = (spin_speed_inner, spin_speed_outer) if webots_turn_command_active == 'turn_left' else (spin_speed_outer, spin_speed_inner)
            if current_time - turn_phase_start_time > MIN_INITIAL_SPIN_DURATION:
                webots_internal_turn_phase = 'SEARCHING_LINE'; turn_phase_start_time = current_time
                print(f"Webots: Turn '{webots_turn_command_active}' state -> SEARCHING_LINE")

        elif webots_internal_turn_phase == 'SEARCHING_LINE':
            search_speed_inner = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.3 # Slower, wider turn for search
            search_speed_outer = FORWARD_SPEED * TURN_SPEED_FACTOR * 0.7
            left_speed, right_speed = (search_speed_inner, search_speed_outer) if webots_turn_command_active == 'turn_left' else (search_speed_outer, search_speed_inner)
            
            line_acquired = on_center_sensor or \
                            (on_left_sensor and on_center_sensor) or \
                            (on_right_sensor and on_center_sensor) or \
                            (webots_turn_command_active == 'turn_left' and on_left_sensor and not on_right_sensor) or \
                            (webots_turn_command_active == 'turn_right' and on_right_sensor and not on_left_sensor)

            if line_acquired:
                webots_internal_turn_phase = 'ADJUSTING_ON_LINE'; turn_phase_start_time = current_time
                print(f"Webots: Turn '{webots_turn_command_active}' state -> ADJUSTING_ON_LINE (Sensors: {line_detected_flags})")
            elif current_time - turn_phase_start_time > MAX_SEARCH_SPIN_DURATION:
                print(f"Webots: Turn '{webots_turn_command_active}' state -> TIMEOUT SEARCHING. Aborting local turn.")
                webots_internal_turn_phase = 'NONE'; webots_turn_command_active = None 
                left_speed, right_speed = 0.0, 0.0 # Stop until ESP32 gives new command

        elif webots_internal_turn_phase == 'ADJUSTING_ON_LINE':
            adj_fwd = FORWARD_SPEED * TURN_ADJUST_SPEED_FACTOR
            adj_turn_dominant = FORWARD_SPEED * TURN_ADJUST_SPEED_FACTOR * 1.5
            adj_turn_recessive = FORWARD_SPEED * TURN_ADJUST_SPEED_FACTOR * 0.5

            if on_center_sensor and not on_left_sensor and not on_right_sensor: # Centered
                left_speed, right_speed = adj_fwd * 0.5, adj_fwd * 0.5 # Crawl/hold
                print(f"Webots: Turn '{webots_turn_command_active}' state: ADJUSTED & Centered. Waiting for ESP 'forward'.")
                # Optionally, could set webots_internal_turn_phase to 'NONE' here if confident,
                # but safer to let ESP32 confirm alignment by sending 'forward'.
            elif on_left_sensor and not on_right_sensor: left_speed, right_speed = adj_turn_recessive, adj_turn_dominant # Turn left
            elif on_right_sensor and not on_left_sensor: left_speed, right_speed = adj_turn_dominant, adj_turn_recessive # Turn right
            elif not any(line_detected_flags): # Lost line during adjustment
                print(f"Webots: Turn '{webots_turn_command_active}' state: Lost line in ADJUST. -> SEARCHING_LINE")
                webots_internal_turn_phase = 'SEARCHING_LINE'; turn_phase_start_time = current_time
            else: left_speed, right_speed = adj_fwd * 0.3, adj_fwd * 0.3 # Other cases, slow down

            if current_time - turn_phase_start_time > MAX_ADJUST_DURATION:
                print(f"Webots: Turn '{webots_turn_command_active}' state -> TIMEOUT ADJUSTING. Aborting local turn.")
                webots_internal_turn_phase = 'NONE'; webots_turn_command_active = None
                left_speed, right_speed = 0.0, 0.0
    else: # Default/unknown ESP32 command
        left_speed, right_speed = 0.0, 0.0 # Stop if command is unknown
        webots_internal_turn_phase = 'NONE'; webots_turn_command_active = None


    left_motor.setVelocity(left_speed); right_motor.setVelocity(right_speed)

    if loop_counter % 3 == 0: update_visualization(robot_world_pose, current_robot_grid_pos, planned_path_grid)
    if loop_counter % 25 == 0:
        status_conn = "🟢 ESP32 CONNECTED" if connection_ok else "🔴 ESP32 DISCONNECTED"
        path_len_info = len(planned_path_grid) if planned_path_grid else "N/A"
        print(f"Sim: {current_time:.1f}s | {status_conn} | ESP: {esp32_current_command.upper()} | "
              f"Grid: {current_robot_grid_pos} | Path: {path_len_info} | Sens: {line_detected_flags} | TurnPhase: {webots_internal_turn_phase}")

if client_sock:
    try: client_sock.close()
    except: pass
if fig: print("🎨 Sim ended. Close plot."); plt.ioff(); plt.show(block=True)
print("✅ Webots Controller Finished.")