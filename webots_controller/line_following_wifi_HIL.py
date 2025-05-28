# line_following_wifi_HIL.py (Webots Controller - Grid Navigation Client - V5 for 13x17 Grid)

from controller import Robot, PositionSensor
import socket
import time
import math
import matplotlib.pyplot as plt

# --- Network Configuration ---
ESP32_IP_ADDRESS = "192.168.53.193"  # <<<<<<<<<<< SET ESP32's ACTUAL IP
ESP32_PORT = 8266
# ---------------------------

# --- Robot Parameters ---
WHEEL_RADIUS = 0.0205; AXLE_LENGTH = 0.052

# --- Grid Map Configuration (CRITICAL: Calibrate these to your Webots world) ---
GRID_ROWS = 13 # Must match ESP32
GRID_COLS = 17 # Must match ESP32
GRID_CELL_SIZE = 0.05  # Example: each grid cell is 5cm x 5cm in Webots

# The origin coordinates define where cell (0,0) is located in the Webots world
# For bottom-left start configuration:
GRID_ORIGIN_X = -0.42   # X-coordinate of the bottom-left corner (cell 0,0)
GRID_ORIGIN_Z = 0.35    # Z-coordinate of the bottom-left corner (cell 0,0)

# Fine-tune offsets if robot position isn't mapping correctly:
X_OFFSET = 0.0  # Fine-tune X offset if needed
Z_OFFSET = 0.0  # Fine-tune Z offset if needed

# Set to True to run in calibration mode (shows grid cells and coordinates)
CALIBRATION_MODE = True
# --- Goal for this Webots instance (sent to ESP32) ---
WEBOTS_TARGET_GOAL_ROW = 12 
WEBOTS_TARGET_GOAL_COL = 16  # End point is on the right in the top row (12,16) 
# -----------------------------------------------------------------------

# --- Robot Control Configuration ---
MAX_SPEED_CELL_NAV = 0.3 # Speed for moving towards next cell center
TURN_SPEED_CELL_NAV = 0.3  # Speed for rotating to face next cell
DIST_TO_CELL_THRESHOLD = GRID_CELL_SIZE * 0.25 
ANGLE_TO_CELL_THRESHOLD = 0.15 # Radians (approx 8.6 degrees)
# -------------------------------------------------------------

# --- Matplotlib Live Plotting ---
plt.ion(); fig_map, ax_map = None,None; robot_path_trail_plot=[]; current_planned_path_nodes_grid = []
plot_dynamic_artists = []
# This MUST match the ESP32's world_grid for plotting obstacles correctly
webots_side_world_grid = [ # Copy of ESP32's world_grid for plotting - updated for bottom-left start
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 0 (bottom)
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 1
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 2
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 3
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 4
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 5
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 6
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 7
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 8
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 9
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # Row 10
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0], # Row 11
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]  # Row 12 (top) - The end point is on the right
]

def world_to_grid_cell(world_x, world_z):
    """
    Convert Webots world coordinates to grid cell coordinates.
    Now includes calibration offsets for fine-tuning.
    """
    # Apply calibration offsets if defined
    adjusted_x = world_x - X_OFFSET
    adjusted_z = world_z - Z_OFFSET
    
    # Calculate grid positions with proper rounding
    col = int(round((adjusted_x - GRID_ORIGIN_X) / GRID_CELL_SIZE))
    row = int(round((GRID_ORIGIN_Z - adjusted_z) / GRID_CELL_SIZE))
    
    # Apply bounds checking to ensure we stay within the grid
    orig_row, orig_col = row, col  # Store original values for debugging
    row = max(0, min(row, GRID_ROWS - 1))
    col = max(0, min(col, GRID_COLS - 1))
    
    # Debug output if values were clamped (indicating potential calibration issue)
    if (orig_row != row or orig_col != col) and webots_loop_counter % 50 == 0:
        print(f"Warning: Grid position clamped from ({orig_row},{orig_col}) to ({row},{col})")
        print(f"World coords: ({world_x:.3f}, {world_z:.3f}), Adjusted: ({adjusted_x:.3f}, {adjusted_z:.3f})")
            
    return row, col

def grid_cell_to_world_center(row, col):
    """
    Convert grid cell coordinates to Webots world coordinates.
    Returns the center point of the specified grid cell.
    """
    # Calculate center of the grid cell in world coordinates
    world_x = GRID_ORIGIN_X + col * GRID_CELL_SIZE + X_OFFSET
    world_z = GRID_ORIGIN_Z - row * GRID_CELL_SIZE + Z_OFFSET
    return world_x, world_z

def update_live_map_plot_grid(current_odom_pose, current_grid_cell_tuple, planned_grid_path=None, obstacle_data=None):
    global fig_map, ax_map, robot_path_trail_plot, plot_dynamic_artists, webots_loop_counter
    if fig_map is None:
        fig_map, ax_map = plt.subplots(figsize=(12,9)); ax_map.set_aspect('equal', 'box') # Adjusted figure size
        ax_map.set_xlabel("World X (m)"); ax_map.set_ylabel("World Z (m)")
        ax_map.set_title(f"Robot Grid Navigation - Origin: ({GRID_ORIGIN_X}, {GRID_ORIGIN_Z}), Offsets: ({X_OFFSET}, {Z_OFFSET})")
        
        # Draw grid lines
        for r_idx_plot in range(GRID_ROWS + 1): 
            wx_start = GRID_ORIGIN_X - 0.5*GRID_CELL_SIZE + X_OFFSET
            wx_end = GRID_ORIGIN_X + (GRID_COLS-0.5)*GRID_CELL_SIZE + X_OFFSET
            wz_line = GRID_ORIGIN_Z - (r_idx_plot - 0.5) * GRID_CELL_SIZE + Z_OFFSET
            ax_map.plot([wx_start, wx_end], [wz_line, wz_line], 'k-', alpha=0.15, lw=0.5, zorder=0)
            
        for c_idx_plot in range(GRID_COLS + 1): 
            wz_start = GRID_ORIGIN_Z + 0.5*GRID_CELL_SIZE + Z_OFFSET
            wz_end = GRID_ORIGIN_Z - (GRID_ROWS-0.5)*GRID_CELL_SIZE + Z_OFFSET
            wx_line = GRID_ORIGIN_X + (c_idx_plot - 0.5) * GRID_CELL_SIZE + X_OFFSET
            ax_map.plot([wx_line, wx_line], [wz_start, wz_end], 'k-', alpha=0.15, lw=0.5, zorder=0)
            
        ax_map.invert_yaxis(); update_live_map_plot_grid.static_artists_drawn = True
        
        # Draw cell coordinates and special markers if in calibration mode
        if CALIBRATION_MODE:
            # Add grid cell coordinates 
            for r in range(0, GRID_ROWS):
                for c in range(0, GRID_COLS):
                    if r % 2 == 0 and c % 2 == 0:  # Only label every other cell to avoid clutter
                        wx, wz = grid_cell_to_world_center(r, c)
                        ax_map.text(wx, wz, f"({r},{c})", ha='center', va='center', fontsize=7, 
                                   color='blue', alpha=0.7, zorder=3)
            
            # Mark origin for reference
            origin_x = GRID_ORIGIN_X + X_OFFSET
            origin_z = GRID_ORIGIN_Z + Z_OFFSET
            ax_map.plot(origin_x, origin_z, 'rx', ms=10, mew=2)
            ax_map.text(origin_x, origin_z, "ORIGIN", color='red', fontsize=10, ha='center', va='bottom')
        
        # Plot obstacles
        if obstacle_data:
            for r_idx_obs in range(len(obstacle_data)):
                for c_idx_obs in range(len(obstacle_data[0])):
                    if obstacle_data[r_idx_obs][c_idx_obs] == 1: 
                        wx_o, wz_o = grid_cell_to_world_center(r_idx_obs, c_idx_obs)
                        rect_o = plt.Rectangle((wx_o-GRID_CELL_SIZE/2, wz_o-GRID_CELL_SIZE/2), 
                                            GRID_CELL_SIZE, GRID_CELL_SIZE, 
                                            fc='dimgray', alpha=0.7, zorder=1)
                        ax_map.add_patch(rect_o)
    for artist in plot_dynamic_artists: artist.remove()
    plot_dynamic_artists = []
    robot_path_trail_plot.append((current_odom_pose['x'], current_odom_pose['y']))
    if len(robot_path_trail_plot) > 500: robot_path_trail_plot.pop(0)
    if len(robot_path_trail_plot)>1: line, = ax_map.plot(*zip(*robot_path_trail_plot), 'c-', lw=1.5, alpha=0.9, zorder=4); plot_dynamic_artists.append(line)
    if planned_grid_path and len(planned_grid_path)>1:
        path_world_x=[]; path_world_z=[]
        for r_node, c_node in planned_grid_path: wx_p, wz_p = grid_cell_to_world_center(r_node, c_node); path_world_x.append(wx_p); path_world_z.append(wz_p)
        line, = ax_map.plot(path_world_x, path_world_z, 'b*--', lw=2, zorder=5, ms=5, alpha=0.8); plot_dynamic_artists.append(line)
    rx,ry_is_z,rth = current_odom_pose['x'],current_odom_pose['y'],current_odom_pose['theta']
    dot, = ax_map.plot(rx,ry_is_z,'ro',ms=7, zorder=6, mec='black'); plot_dynamic_artists.append(dot)
    arrow_len = GRID_CELL_SIZE*0.45; arrow = ax_map.arrow(rx,ry_is_z,arrow_len*math.cos(rth),arrow_len*math.sin(rth),head_width=arrow_len*0.4,head_length=arrow_len*0.6,fc='maroon',ec='maroon',zorder=7,lw=1.5); plot_dynamic_artists.append(arrow)
    if current_grid_cell_tuple:
        curr_cell_wx,curr_cell_wz = grid_cell_to_world_center(current_grid_cell_tuple[0],current_grid_cell_tuple[1])
        rect_curr=plt.Rectangle((curr_cell_wx-GRID_CELL_SIZE/2,curr_cell_wz-GRID_CELL_SIZE/2),GRID_CELL_SIZE,GRID_CELL_SIZE,edgecolor='gold',facecolor='yellow',alpha=0.3,zorder=0,lw=1.5); ax_map.add_patch(rect_curr);plot_dynamic_artists.append(rect_curr)
    if not hasattr(update_live_map_plot_grid,'legend_added_flag') and (len(robot_path_trail_plot)>1 or (planned_grid_path and len(planned_grid_path)>1)):
        ax_map.legend(['Odometry Trail','Planned Path','Current Position'],loc='upper left',fontsize='x-small',bbox_to_anchor=(1.01,1)); fig_map.tight_layout(rect=[0,0,0.80,1]); update_live_map_plot_grid.legend_added_flag=True # Adjusted rect for legend
    if not hasattr(update_live_map_plot_grid,'bounds_set_flag') or webots_loop_counter % 200 == 1 :
        min_wx_plot=GRID_ORIGIN_X-GRID_CELL_SIZE*1.5;max_wx_plot=GRID_ORIGIN_X+(GRID_COLS+0.5)*GRID_CELL_SIZE # Extended bounds slightly
        min_wz_plot_axis=GRID_ORIGIN_Z-(GRID_ROWS+0.5)*GRID_CELL_SIZE;max_wz_plot_axis=GRID_ORIGIN_Z+GRID_CELL_SIZE*1.5
        ax_map.set_xlim(min_wx_plot,max_wx_plot);ax_map.set_ylim(max_wz_plot_axis, min_wz_plot_axis)
        update_live_map_plot_grid.bounds_set_flag = True
    plt.draw(); plt.pause(0.001)

# --- Initialize Robot & Devices ---
# ... (Identical to V12) ...
robot = Robot(); timestep = int(robot.getBasicTimeStep())
robot_pose = {'x':0.0,'y':0.0,'theta':0.0}; prev_left_enc=0.0; prev_right_enc=0.0; first_odom=True
left_motor=robot.getDevice('left wheel motor'); right_motor=robot.getDevice('right wheel motor')
if not (left_motor and right_motor): print("Motors missing"); exit(1)
for m in (left_motor,right_motor): m.setPosition(float('inf')); m.setVelocity(0.0)
print("✅ Webots: Motors initialized.")
left_enc=robot.getDevice('left wheel sensor'); right_enc=robot.getDevice('right wheel sensor')
encoders_ok = False
if left_enc and right_enc: left_enc.enable(timestep);right_enc.enable(timestep);encoders_ok=True;print("✅ Webots: Encoders enabled.")
else: print("⚠️ Webots: Encoders missing.")
# Ground sensors are not explicitly used by Webots for grid navigation logic in this pure grid version.

# --- Network Client ---
# ... (setup_client function - identical to V12, no INIT message sent) ...
client_sock=None; conn_ok=False; last_conn_attempt=0
def setup_client():
    global client_sock,conn_ok,last_conn_attempt
    if client_sock: 
        try:
            client_sock.close() 
        except:pass
    client_sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM); client_sock.settimeout(3.0)
    print(f"Webots: Connect ESP32 @ {ESP32_IP_ADDRESS}:{ESP32_PORT}...")
    try: client_sock.connect((ESP32_IP_ADDRESS,ESP32_PORT));client_sock.settimeout(1.0);conn_ok=True;print("✅ Webots: Connected.")
    except Exception as e: print(f"⚠️ Webots: Connect fail: {e}");conn_ok=False
    last_conn_attempt=time.time(); return conn_ok
setup_client()

# --- Main Loop ---
# ... (Identical to V12, including odometry, POS sending, GOTO execution, plotting call) ...
webots_loop_counter = 0; target_cell_r, target_cell_c = -1, -1; robot_is_moving_to_target = False
# Define a consistent start position for the bottom-left layout
r_start_sim, c_start_sim = 0, 0  # Bottom-left corner (updated for new layout)
theta_start_sim = math.pi/2  # 90 degrees, facing up for bottom-left start
robot_pose['x'], robot_pose['y'] = grid_cell_to_world_center(r_start_sim, c_start_sim)
robot_pose['theta'] = theta_start_sim
first_odom = True 
print(f"\nWebots: Grid Navigation. Odometry starts relative to grid ({r_start_sim},{c_start_sim}) facing {math.degrees(theta_start_sim):.1f} deg world.");

while robot.step(timestep) != -1:
    webots_loop_counter += 1
    
    # Use only encoder-based odometry for position tracking
    # (getPosition isn't available for the robot in this Webots version)
    if encoders_ok:
        curr_L=left_enc.getValue(); curr_R=right_enc.getValue()
        if first_odom:
            prev_left_enc=curr_L; prev_right_enc=curr_R; first_odom=False
        else:
            dL=(curr_L-prev_left_enc)*WHEEL_RADIUS; dR=(curr_R-prev_right_enc)*WHEEL_RADIUS
            if abs(dL) > 0.0001 or abs(dR) > 0.0001:  # Only update with encoder if wheels actually moved
                dC=(dL+dR)/2.0; dTh=(dR-dL)/AXLE_LENGTH
                robot_pose['x']+=dC*math.cos(robot_pose['theta']+dTh/2.0)
                robot_pose['y']+=dC*math.sin(robot_pose['theta']+dTh/2.0) # y is world Z
                robot_pose['theta']+=dTh; robot_pose['theta']=math.atan2(math.sin(robot_pose['theta']),math.cos(robot_pose['theta']))
            prev_left_enc=curr_L; prev_right_enc=curr_R
    # Map world coordinates to grid cell
    current_r_mapped, current_c_mapped = world_to_grid_cell(robot_pose['x'], robot_pose['y'])
    
    # Print debug position occasionally to monitor the mapping
    if webots_loop_counter % 100 == 0:
        print(f"Robot pos: World({robot_pose['x']:.3f}, {robot_pose['y']:.3f}) -> Grid({current_r_mapped}, {current_c_mapped})")
    
    if not conn_ok: # Connection & POS send ...
        if time.time()-last_conn_attempt>5.0:setup_client()
        left_motor.setVelocity(0.0);right_motor.setVelocity(0.0);time.sleep(0.1);continue        # Add a 'last known position' to track when it actually changes
    static_pos_check = getattr(world_to_grid_cell, 'last_known_position', None)
    current_pos = (current_r_mapped, current_c_mapped)
    
    # Check if position actually changed
    if static_pos_check != current_pos:
        world_to_grid_cell.last_known_position = current_pos
        if static_pos_check is not None:  # Don't log the first time
            print(f"Position CHANGED from {static_pos_check} to {current_pos}")
    
    # Send current position and target goal to ESP32
    pos_msg = f"POS:{current_r_mapped},{current_c_mapped},{WEBOTS_TARGET_GOAL_ROW},{WEBOTS_TARGET_GOAL_COL}\n"
    try:client_sock.sendall(pos_msg.encode('utf-8'))
    except Exception as e:print(f"❌ Send POS Error:{e}");conn_ok=False;continue
    cmd_from_esp32="";# Recv command ...
    try:
        client_sock.settimeout(0.05);data_bytes=client_sock.recv(256);client_sock.settimeout(1.0)
        if not data_bytes: print("⚠️ ESP32 closed conn.");conn_ok=False
        else:
            full_resp=data_bytes.decode('utf-8');msgs=full_resp.splitlines();temp_goto_cmd=None
            for msg_part in msgs:
                msg=msg_part.strip();
                if not msg:continue
                if msg.startswith("PATH_GRID:"):
                    try:p_str=msg.split(":",1)[1];current_planned_path_nodes_grid=[tuple(map(int,cell.split(','))) for cell in p_str.split(';')]
                    except Exception as ep:print(f"Webots Warn:Parse PATH_GRID fail:'{msg}',E:{ep}");current_planned_path_nodes_grid=[]
                elif msg.startswith("GOTO:"):
                    try:coords=msg.split(":",1)[1].split(',');target_cell_r,target_cell_c=int(coords[0]),int(coords[1]);robot_is_moving_to_target=True;temp_goto_cmd=msg
                    except Exception as eg:print(f"Webots Warn:Parse GOTO fail:'{msg}',E:{eg}");robot_is_moving_to_target=False
                elif msg=="stop":robot_is_moving_to_target=False;target_cell_r,target_cell_c=-1,-1;temp_goto_cmd=msg
                elif msg:print(f"⚠️ Webots:Unhandled msg ESP32:'{msg}'")
            if temp_goto_cmd: cmd_from_esp32 = temp_goto_cmd # Prioritize GOTO or STOP
        if not conn_ok:continue
    except socket.timeout: pass
    except Exception as e:print(f"❌ Recv Error:{e}");conn_ok=False
    if not conn_ok:continue
    left_speed,right_speed=0.0,0.0 # Execute movement ...
    if robot_is_moving_to_target and target_cell_r!=-1:
        target_wx,target_wz=grid_cell_to_world_center(target_cell_r,target_cell_c)
        dx=target_wx-robot_pose['x'];dz=target_wz-robot_pose['y']
        dist_target=math.sqrt(dx*dx+dz*dz);angle_target=math.atan2(dz,dx)
        angle_diff=math.atan2(math.sin(angle_target-robot_pose['theta']),math.cos(angle_target-robot_pose['theta']))
        if dist_target>DIST_TO_CELL_THRESHOLD:
            if abs(angle_diff)>ANGLE_TO_CELL_THRESHOLD:
                turn_val=max(-TURN_SPEED_CELL_NAV,min(TURN_SPEED_CELL_NAV,angle_diff*2.0)) # P-control for turn
                left_speed=-turn_val;right_speed=turn_val
            else:left_speed=MAX_SPEED_CELL_NAV;right_speed=MAX_SPEED_CELL_NAV
        else:robot_is_moving_to_target=False;left_speed,right_speed=0.0,0.0 # Arrived
    elif cmd_from_esp32 == "stop": left_speed,right_speed=0.0,0.0;robot_is_moving_to_target=False
        
    left_motor.setVelocity(left_speed);right_motor.setVelocity(right_speed)
    if encoders_ok and webots_loop_counter%5==1:update_live_map_plot_grid(robot_pose,(current_r_mapped,current_c_mapped),current_planned_path_nodes_grid,webots_side_world_grid)

if client_sock:client_sock.close();print("ℹ️ Webots:Client sock closed.")
if fig_map:plt.ioff();plt.show(block=True)
left_motor.setVelocity(0.0);right_motor.setVelocity(0.0);print("ℹ️ Webots:Controller finished.")