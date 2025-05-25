cat <<'EOF' > Robotics_Lab7_HIL_Dijkstra/README.md
# Lab 7: Hardware-in-the-Loop Simulation with Dijkstra Path Planning

This project implements a Hardware-in-the-Loop (HIL) simulation where an ESP32 microcontroller, programmed in MicroPython, controls a robot in the Webots simulator to navigate the shortest path on a predefined map using Dijkstra's algorithm. Communication between the ESP32 and Webots is achieved over Wi-Fi using TCP/IP sockets.

## Team Members
* [Your Name(s)]

## Features Implemented
* **Hardware-in-the-Loop (HIL) Simulation:** ESP32 controls the Webots robot. [cite: 26, 27]
* **Wireless Communication:** Wi-Fi (TCP/IP) between ESP32 and Webots. [cite: 36]
* **Path Planning on ESP32:** Dijkstra's algorithm computes the shortest path. [cite: 28]
* **Line Following:** Reactive line following for navigating segments between nodes.
* **Automated Navigation:** Finite State Machine on ESP32 manages path planning, junction maneuvering, and segment following.

## Hardware Requirements
* ESP32 development board (e.g., ESP32-WROOM-32 based).
* Computer capable of running Webots and Python.
* Wi-Fi network accessible by both the ESP32 and the computer.

## Software Requirements & Dependencies
* **Webots:** Version R2023a or newer. [cite: 11]
* **Python (for Webots controller):** Python 3.7+ (standard `socket` and `time` modules used).
* **MicroPython (for ESP32):** Version X.X.X flashed onto the ESP32 (specify your version). Standard `network`, `socket`, `time`, `machine.Pin` modules used.

## Setup Instructions

### 1. ESP32 Setup
1.  Modify `esp32_code/main.py`:
    * Update `WIFI_SSID` and `WIFI_PASSWORD` with your Wi-Fi credentials.
    * Review and set `START_NODE`, `GOAL_NODE`, and `INITIAL_ORIENTATION` for the desired path.
    * (Optional) Adjust `ONBOARD_LED_PIN` if your board's LED is on a different GPIO.
2.  Upload `esp32_code/main.py` to your ESP32 using a MicroPython IDE like Thonny.
3.  Run the script on the ESP32. It will attempt to connect to Wi-Fi and then print its IP address and start the TCP server (e.g., "TCP Server listening on 192.168.1.XXX:8266"). **Note this IP address.**
4.  You can keep Thonny connected to the ESP32's REPL to view debug `print()` statements.

### 2. Webots Setup
1.  Ensure you have the Webots world file (e.g., `RaFLite.wbt` from the lab, or your custom one, placed in the `world/` directory if you include it).
2.  Open the Webots world.
3.  Assign the `webots_controller/line_following_wifi_HIL.py` script as the controller for your e-puck robot.
4.  Modify `webots_controller/line_following_wifi_HIL.py`:
    * **Crucially, update `ESP32_IP_ADDRESS`** with the IP address obtained from your ESP32 in the previous step.
    * Ensure `ESP32_PORT` (default 8266) matches the ESP32 script.
    * Tune `LINE_THRESHOLD` and motor speeds (`MAX_FORWARD_SPEED`, `SHARP_TURN_ROTATE_SPEED`, etc.) as needed. The `CYCLES_FOR_..._TURN` constants in the ESP32 script are highly dependent on these Webots speed settings.

## How to Run the Simulation (Reproducibility [cite: 24])
1.  Ensure the ESP32 is running `main.py`, connected to Wi-Fi, and its TCP server is listening (as confirmed by its serial output).
2.  Ensure the Webots controller script (`line_following_wifi_HIL.py`) has the correct ESP32 IP address.
3.  Run the Webots simulation.
4.  The Webots console should show it attempting to connect and then "Connected to ESP32 server...".
5.  The ESP32 console should show "Webots connected from: ...".
6.  The ESP32 will then start path planning and sending commands to the robot.

## Tuning Maneuver Cycles (Important for Performance)
In `esp32_code/main.py`, the following constants control how long the ESP32 commands specific maneuvers at junctions:
* `CYCLES_FOR_90_DEG_TURN`
* `CYCLES_FOR_STRAIGHT_JUNCTION_CROSSING`
* `CYCLES_FOR_180_DEG_TURN`
These values represent the number of ESP32 control cycles (each approx. 0.02s) that a maneuver command will be sent. They **must be tuned** by observing the robot in Webots. For example, if `CYCLES_FOR_90_DEG_TURN = 15` and the robot over-rotates, reduce this value. If it under-rotates, increase it. This depends on the `SHARP_TURN_ROTATE_SPEED` set in the Webots script.

## Known Issues / Limitations
* Turn precision at junctions depends on careful tuning of maneuver cycle counts and Webots motor speeds.
* The map of the environment is currently hardcoded into the ESP32 script.
* No dynamic obstacle avoidance or re-planning is implemented in this version. [Requirement for full grade: [cite: 31]]
* No real-time path visualization is implemented. [Requirement for full grade: [cite: 34]]

## Future Work
* Implement dynamic obstacle detection using proximity sensors and path re-planning.
* Add a path visualization component.
* Allow dynamic input of Start/Goal nodes (e.g., via a simple web interface on ESP32 or other means).
EOF
