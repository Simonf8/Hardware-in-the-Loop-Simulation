# line_following_internal_logic.py
#
# Reads three ground (line) sensors from an E-puck in Webots,
# implements internal line-following logic, and drives the wheels.

from controller import Robot

# 1) Initialize Robot and time step
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2) Motors (velocity control)
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

if left_motor is None or right_motor is None:
    print("❌ Error: One or both motors not found. Check device names in Webots (e.g., 'left wheel motor', 'right wheel motor').")
    exit(1)
else:
    print("✅ Motors found and initialized.")
    for m in (left_motor, right_motor):
        m.setPosition(float('inf'))  # Enable velocity control
        m.setVelocity(0.0)           # Start stopped

# 3) Ground sensors (line detectors)
sensor_names = ['gs0', 'gs1', 'gs2']  # Expected names: left, center, right
sensors = []
for name in sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None:
        print(f"⚠️ Warning: Ground sensor device '{name}' not found.")
    else:
        sensor.enable(timestep)
        sensors.append(sensor)
        print(f"✅ Ground sensor '{name}' enabled.")

if len(sensors) != 3: # Ensure all three sensors are critical for this logic
    print(f"❌ Error: Expected 3 ground sensors, but found {len(sensors)}. Check your PROTO/device names.")
    exit(1)
else:
    print("✅ All 3 ground sensors initialized.")

# 4) Line-vs-no-line threshold (tune in your simulation based on sensor readings)
# Typically, lower values mean a dark line is detected by IR sensors.
LINE_THRESHOLD = 600 # Adjust this value based on your simulation environment and sensor readings.
                     # Values below threshold are considered "on the line".

# 5) State machine default and parameters
current_state = 'stop' # Initial state
last_turn_direction = 'left' # For a slightly more intelligent search, could alternate or use last known

# Robot speeds (adjust as needed)
MAX_FORWARD_SPEED = 3.0       # Speed when moving straight
TURN_PIVOT_SPEED = 2.5        # Speed of the outer wheel when pivoting (inner wheel is 0)
                                # Or, for on-the-spot turns, this could be used.
SEARCH_ROTATE_SPEED = 1.5     # Speed for in-place rotation during search

# 6) Main control loop
while robot.step(timestep) != -1:

    # a) Read sensors and binarize
    raw_sensor_values = [s.getValue() for s in sensors]
    # print(f"DEBUG: Raw sensor values: {raw_sensor_values}") # Uncomment for tuning LINE_THRESHOLD

    # Binarize: True (1) if sensor is over the line (dark), False (0) otherwise
    # This assumes lower sensor values correspond to a darker surface (the line)
    # s_left, s_middle, s_right
    on_line_flags = [value < LINE_THRESHOLD for value in raw_sensor_values]
    s_left, s_middle, s_right = on_line_flags

    # print(f"DEBUG: Binarized sensors (L,M,R): [{int(s_left)}, {int(s_middle)}, {int(s_right)}]") # Uncomment for debugging

    # b) Internal line-following logic to determine current_state
    if not s_left and s_middle and not s_right: # Pattern: 010 (Only middle on line)
        current_state = 'forward'
    elif s_left and s_middle and not s_right:   # Pattern: 110 (Left and Middle on line) - Gentle left turn
        current_state = 'turn_left_gentle' # Robot is slightly too right, needs to steer left
    elif s_left and not s_middle and not s_right: # Pattern: 100 (Only Left on line) - Sharper left turn
        current_state = 'turn_left_sharp' # Robot is too right, needs to steer left
    elif not s_left and s_middle and s_right:   # Pattern: 011 (Middle and Right on line) - Gentle right turn
        current_state = 'turn_right_gentle' # Robot is slightly too left, needs to steer right
    elif not s_left and not s_middle and s_right: # Pattern: 001 (Only Right on line) - Sharper right turn
        current_state = 'turn_right_sharp' # Robot is too left, needs to steer right
    elif s_left and s_middle and s_right:       # Pattern: 111 (All sensors on line - e.g., intersection or thick line)
        current_state = 'forward' # Or 'stop' depending on desired behavior at intersections
    elif not s_left and not s_middle and not s_right: # Pattern: 000 (No line detected)
        current_state = 'search'
        # Simple search: toggle last turn direction to alternate search spins
        # if last_turn_direction == 'left':
        #     last_turn_direction = 'right'
        # else:
        #     last_turn_direction = 'left'
    # else: # Other patterns like 101 (left and right, but not middle)
          # Could be treated as 'search' or keep previous state.
          # For simplicity, if not explicitly handled, might fall into 'search' or last known state.
          # current_state = 'search' # Or maintain previous state by not changing it

    # c) Map current_state to wheel speeds
    left_speed, right_speed = 0.0, 0.0 # Default to stop

    if current_state == 'forward':
        left_speed, right_speed = MAX_FORWARD_SPEED, MAX_FORWARD_SPEED
    elif current_state == 'turn_left_gentle': # Gentle left: reduce left speed slightly or pivot slowly
        left_speed, right_speed = MAX_FORWARD_SPEED * 0.5, MAX_FORWARD_SPEED
        last_turn_direction = 'left'
    elif current_state == 'turn_left_sharp':  # Sharp left: pivot around left wheel or spin
        left_speed, right_speed = 0.0, TURN_PIVOT_SPEED
        # Alternative for sharper on-spot turn: -SEARCH_ROTATE_SPEED, SEARCH_ROTATE_SPEED
        last_turn_direction = 'left'
    elif current_state == 'turn_right_gentle':# Gentle right: reduce right speed slightly or pivot slowly
        left_speed, right_speed = MAX_FORWARD_SPEED, MAX_FORWARD_SPEED * 0.5
        last_turn_direction = 'right'
    elif current_state == 'turn_right_sharp': # Sharp right: pivot around right wheel or spin
        left_speed, right_speed = TURN_PIVOT_SPEED, 0.0
        # Alternative for sharper on-spot turn: SEARCH_ROTATE_SPEED, -SEARCH_ROTATE_SPEED
        last_turn_direction = 'right'
    elif current_state == 'search':
        # Rotate in place based on last known direction or a fixed direction
        if last_turn_direction == 'left': # If we were trying to turn left and lost the line, continue left
            left_speed, right_speed = -SEARCH_ROTATE_SPEED, SEARCH_ROTATE_SPEED # Spin left
        else: # If we were trying to turn right
            left_speed, right_speed = SEARCH_ROTATE_SPEED, -SEARCH_ROTATE_SPEED # Spin right
        # A simpler search: always spin one way, e.g., SEARCH_ROTATE_SPEED, -SEARCH_ROTATE_SPEED
    elif current_state == 'stop': # Explicit stop state
        left_speed, right_speed = 0.0, 0.0
    else:
        print(f"🚨 Webots: Unknown state '{current_state}'. Stopping robot.")
        left_speed, right_speed = 0.0, 0.0
        current_state = 'stop'

    # d) Apply speeds to motors
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # print(f"State: {current_state}, Speeds: L={left_speed:.2f}, R={right_speed:.2f}") # DEBUG

# Cleanup (Webots handles controller termination well, so this is mostly for completeness)
print("ℹ️ Webots controller finished.")
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)