# Lab 7: HIL Simulation with Dijkstra Path Planning

This project demonstrates a Hardware-in-the-Loop (HIL) simulation where an ESP32 (running MicroPython) controls a Webots robot over Wi-Fi. The robot navigates a map using Dijkstra's shortest path algorithm.


## Features
- ESP32 controls the robot in Webots (HIL).
- Wi-Fi communication via TCP/IP.
- Dijkstra path planning on ESP32.
- Reactive line following between path nodes.
- FSM handles navigation logic.

## Requirements
### Hardware
- ESP32 dev board
- Computer with Webots
- Wi-Fi network

### Software
- Webots R2023a+
- Python 3.7+ (for Webots controller)
- MicroPython (flashed to ESP32)

## Setup

### ESP32
1. Edit `esp32_code/main.py`:
   - Set your Wi-Fi credentials
   - Define `START_NODE`, `GOAL_NODE`, etc.
2. Upload to ESP32 using Thonny or similar.
3. Run the script; note the printed IP.

### Webots
1. Place `RaFLite.wbt` in `world/`.
2. Assign `webots_controller/line_following_wifi_HIL.py` to the robot.
3. Edit the controller script:
   - Update `ESP32_IP_ADDRESS`
   - Tune motor speeds and thresholds if needed

## Running the Simulation
1. Run `main.py` on ESP32 (ensure Wi-Fi + server running)
2. Start Webots simulation
3. Connection is established automatically
4. The robot follows the planned path

## Notes on Tuning
Adjust `CYCLES_FOR_...` in `main.py` based on Webots motor speeds:
- Over-rotating? Decrease cycle count.
- Under-rotating? Increase it.

## Next tasks
- Hardcoded map
- obstacle avoidance or re-planning
- real-time visualization

## Future Ideas
- Add obstacle detection Sand path re-planning
- Visualize path in real-time
- Select start/goal via web interface
EOF
