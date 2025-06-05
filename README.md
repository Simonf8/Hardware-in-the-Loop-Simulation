<div align="center">

# HARDWARE-IN-THE-LOOP NAVIGATION SYSTEM
    PLEASE DON'T COPY EVERYTHING AND HAND IT IN AS YOUR WORK, USE IT AS INSPIRATION
    IF IT DOES'T WORK FOR YOU EITHER BLAME YOUR LAPTOP OR WIFI AND MOVE ON

## LIVE DEMONSTRATION

[![Demo](https://img.shields.io/badge/▶_WATCH_LIVE_DEMO-DIJKSTRA_NAVIGATION-2D1B69?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/QRNA4Zo2aBg)


# ESP32 MicroPython Project

<div align="center">

![STATUS](https://img.shields.io/badge/STATUS-ACTIVE-00E5FF?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjAiIGhlaWdodD0iMjAiIHZpZXdCb3g9IjAgMCAyMCAyMCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPGNpcmNsZSBjeD0iMTAiIGN5PSIxMCIgcj0iOCIgZmlsbD0iIzAwRTVGRiIvPgo8L3N2Zz4K)
![ESP32](https://img.shields.io/badge/ESP32-MICROPYTHON-4A148C?style=for-the-badge&logo=micropython&logoColor=white)
![WEBOTS](https://img.shields.io/badge/WEBOTS-R2023A+-6A0DAD?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTEyIDJMMTMuMDkgOC4yNkwyMCA5TDEzLjA5IDE1Ljc0TDEyIDIyTDEwLjkxIDE1Ljc0TDQgOUwxMC45MSA4LjI2TDEyIDJaIiBmaWxsPSJ3aGl0ZSIvPgo8L3N2Zz4K)
![ALGORITHMS](https://img.shields.io/badge/ALGORITHMS-3_TYPES-E91E63?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTEyIDJMMTMuMDkgOC4yNkwyMCA5TDEzLjA5IDE1Ljc0TDEyIDIyTDEwLjkxIDE1Ljc0TDQgOUwxMC45MSA4LjI2TDEyIDJaIiBmaWxsPSJ3aGl0ZSIvPgo8L3N2Zz4K)

![THONNY](https://img.shields.io/badge/IDE-THONNY_4.1.7-00A86B?style=for-the-badge&logo=python&logoColor=white)
![MICROPYTHON](https://img.shields.io/badge/FIRMWARE-MICROPYTHON-FF6B35?style=for-the-badge&logo=micropython&logoColor=white)
![PYTHON](https://img.shields.io/badge/PYTHON-3.12.10-3776AB?style=for-the-badge&logo=python&logoColor=white)

</div>

---

##  Tech Stack

| Component | Technology | Version |
|-----------|------------|---------|
| **IDE** | Thonny | 4.1.7 |
| **Firmware** | MicroPython | Latest |
| **Platform** | Linux | - | I USE ARCH BTW
| **Python** | Python | 3.12.10 |
| **GUI** | Tk | 8.6.15 |
|  **Simulator** | Webots | R2023a+ |

##  Project Features

<div align="center">

###  **ESP32 Development**
*Real-time embedded programming with MicroPython firmware*

### **Smart Algorithms** 
*Three different algorithm implementations for optimization*

###  **Simulation Ready**
*Full Webots integration for testing and validation*

###  **Live Development**
*Interactive coding with Thonny's powerful debugging tools*

</div>

## Quick Setup

```bash
# 1. Flash MicroPython firmware to ESP32
esptool.py --chip esp32 erase_flash
esptool.py --chip esp32 write_flash -z 0x1000 micropython.bin

# 2. Open Thonny IDE
# 3. Configure interpreter: Tools > Options > Interpreter
# 4. Select "MicroPython (ESP32)" and your COM port
# 5. Ready to code! 
```

##  Hardware Requirements

- 🔹 **ESP32** development board (DevKit v1 recommended)
- 🔹 USB cable for programming
- 🔹 Sensors/actuators as per project specs
- 🔹 Breadboard and jumper wires

## 💻 Software Dependencies

- 🔸 **Thonny IDE** 4.1.7+
- 🔸 **MicroPython firmware** for ESP32
- 🔸 **Webots** R2023a+ (simulation)
- 🔸 **esptool.py** (firmware flashing)

## Usage

### For ESP32 Development:
1. Connect your ESP32 to your computer
2. Open Thonny and select MicroPython (ESP32) interpreter
3. Load `main.py` and run directly on hardware
4. Monitor output in Thonny's shell

### For Simulation:
1. Open Webots R2023a+
2. Load the world file from `simulation/`
3. Run the simulation with your algorithms
4. Compare results with real hardware

## Contributing

Feel free to contribute to this project! Whether it's:
- Bug fixes
- New features  
- Documentation improvements
- Test cases

## License

This project is open source - feel free to use and modify!

---

<div align="center">



** If this project helped you, give it a star!**

</div>

---

*Developed with Thonny IDE - Made at the University of Tartu, Estonia*

```ascii
╔══════════════════════════════════════════════════════════════╗
║                     REAL HARDWARE CONTROLS VIRTUAL ROBOT     ║
║                                                              ║
║  ESP32 ←→ WiFi ←→ Webots  |  Live Pathfinding  |Real-time HIL║
╚══════════════════════════════════════════════════════════════╝
```


</div>

---

## ⚡ SYSTEM OVERVIEW

<table align="center" width="100%">
<tr>
<td width="50%" align="center">

### REAL HARDWARE
```
┌─────────────────┐
│     ESP32       │
│   BRAIN UNIT    │
│                 │
│ ▪ MicroPython   │
│ ▪ WiFi Stack    │
│ ▪ Path Planning │
│ ▪ Algorithms    │
└─────────────────┘
```

</td>
<td width="50%" align="center">

### VIRTUAL ENVIRONMENT
```
┌─────────────────┐
│     WEBOTS      │
│  SIMULATION     │
│                 │
│ ▪ Physics Eng   │
│ ▪ Sensors       │
│ ▪ Visualization │
│ ▪ Real-time     │
└─────────────────┘
```

</td>
</tr>
</table>

---

## 🧠 ALGORITHM ARSENAL

<table align="center">
<tr>
<td width="33%" align="center">

### DIJKSTRA CLASSIC
```
██████████████████████
█ GUARANTEED OPTIMAL █
██████████████████████
```
**Perfect Education Tool**
- 100% Optimal Paths
- O((V+E)log V) Complexity
- Educational Excellence
- `Dijkstra.py`

</td>
<td width="33%" align="center">

### A* LIGHTNING
```
████████████████
█ SPEED DEMON █
████████████████
```
**Heuristic Powerhouse**
- 3x Faster Planning
- Goal-Directed Search
- Memory Efficient
- `A_algorithm.py`

</td>
<td width="33%" align="center">

### D* LITE ADAPTIVE
```
████████████████████
█ DYNAMIC EVOLUTION █
████████████████████
```
**AI Learning System**
- Real-time Replanning
- Obstacle Adaptation
- Self-Improving
- `D_algorithm.py`

</td>
</tr>
</table>

---

## 🚀 QUICK START FLOWCHART

```mermaid
graph TD
    A["🔹 CLONE REPOSITORY"] --> B["⚙️ FLASH ESP32"]
    B --> C["📝 EDIT WiFi CONFIG"]
    C --> D["⚡ CHOOSE ALGORITHM"]
    D --> E["🎮 LAUNCH WEBOTS"]
    E --> F["🎯 WATCH MAGIC HAPPEN"]
    
    D --> D1["📚 Dijkstra.py<br/>→ Education"]
    D --> D2["⚡ A_algorithm.py<br/>→ Speed"]
    D --> D3["🧠 D_algorithm.py<br/>→ Dynamic"]
    
    style A fill:#2D1B69,stroke:#00E5FF,stroke-width:3px,color:#fff
    style F fill:#4A148C,stroke:#E91E63,stroke-width:3px,color:#fff
    style D1 fill:#1565C0,color:#fff
    style D2 fill:#0D47A1,color:#fff
    style D3 fill:#6A0DAD,color:#fff
```

---

## ⚡ INSTALLATION MATRIX

<table align="center">
<tr>
<th>STEP</th>
<th>WINDOWS</th>
<th>MACOS</th>
<th>LINUX</th>
</tr>
<tr>
<td><strong>WEBOTS</strong></td>
<td>Download .exe → Install</td>
<td>Download .dmg → Drag</td>
<td><code>wget webots.deb</code></td>
</tr>
<tr>
<td><strong>PYTHON</strong></td>
<td><code>pip install matplotlib numpy</code></td>
<td><code>pip3 install matplotlib numpy</code></td>
<td><code>sudo apt install python3-pip</code></td>
</tr>
<tr>
<td><strong>ESP32</strong></td>
<td colspan="3" align="center">Flash MicroPython via Thonny IDE</td>
</tr>
</table>

---

## 🎯 SYSTEM ARCHITECTURE

```mermaid
graph LR
    subgraph "🔮 HARDWARE LAYER"
        A[ESP32<br/>🧠 Algorithm Engine]
        B[WiFi Module<br/>📡 Bridge]
    end
    
    subgraph "🌐 VIRTUAL LAYER"
        C[Webots Robot<br/>🤖 Avatar]
        D[Physics Engine<br/>⚡ Reality]
    end
    
    subgraph "📊 INTELLIGENCE"
        E[Path Planner<br/>🧭 Navigator]
        F[Live Dashboard<br/>📈 Monitor]
    end
    
    A -->|JSON/TCP| C
    C -->|Sensor Data| E
    E -->|Commands| A
    D --> F
    
    style A fill:#2D1B69,stroke:#00E5FF,stroke-width:3px,color:#fff
    style C fill:#4A148C,stroke:#E91E63,stroke-width:3px,color:#fff
    style E fill:#6A0DAD,stroke:#CFD8DC,stroke-width:3px,color:#fff
```

---

## ⚙️ CONFIGURATION SIMPLIFIED

### ESP32 SETUP
```python
# EDIT YOUR CHOSEN ALGORITHM FILE
WIFI_SSID = 'Your_Network'
WIFI_PASSWORD = 'Your_Password'
# UPLOAD TO ESP32 AS main.py
```

### WEBOTS SETUP
```python
# EDIT: webots_controller/line_following_wifi_HIL.py
ESP32_IP_ADDRESS = "192.168.x.x"  # FROM ESP32 OUTPUT
```

---

## 🏆 PERFORMANCE MATRIX

<div align="center">

| ALGORITHM | SPEED | QUALITY | MEMORY | ADAPTABILITY |
|-----------|-------|---------|--------|--------------|
| **DIJKSTRA** | `█░░` | `███` | `███` | `░░░` |
| **A\*** | `███` | `███` | `█░░` | `░░░` |
| **D\* LITE** | `██░` | `██░` | `██░` | `███` |

</div>

---

## 🔧 TROUBLESHOOTING QUICK FIXES

```mermaid
graph TD
    A["⚠️ PROBLEM DETECTED"] --> B{"NETWORK ISSUE?"}
    A --> C{"ROBOT NOT MOVING?"}
    A --> D{"SLOW PLANNING?"}
    
    B -->|YES| B1["✓ Check WiFi 2.4GHz<br/>✓ Verify IP Address<br/>✓ Same Network"]
    C -->|YES| C1["✓ Check Grid Config<br/>✓ Sensor Threshold<br/>✓ Path Calculated"]
    D -->|YES| D1["✓ Use A* Algorithm<br/>✓ Reduce Grid Size<br/>✓ Increase Interval"]
    
    style A fill:#E91E63,stroke:#fff,stroke-width:3px,color:#fff
    style B1 fill:#4A148C,color:#fff
    style C1 fill:#1565C0,color:#fff
    style D1 fill:#6A0DAD,color:#fff
```

---

## 🎮 PROJECT STRUCTURE

```
HIL-Navigation-System/
├── 🎬 demo.mp4                    # Live demonstration
├── 📸 track_layout.png            # Circuit layout
├── 🧠 esp32_code/
│   ├── ⚡ Dijkstra.py            # Optimal pathfinding
│   ├── 🚀 A_algorithm.py         # Speed-focused
│   └── 🧬 D_algorithm.py         # Adaptive learning
├── 🎮 webots_controller/
│   └── line_following_wifi_HIL.py # Universal controller
└── 🌍 world/
    └── RaFLite.wbt               # Simulation environment
```

---

## 🌟 ADVANCED FEATURES UNLOCKED

<table align="center">
<tr>
<td width="50%">

### REAL-TIME CAPABILITIES
- **Live Obstacle Detection**
- **Dynamic Path Replanning**  
- **Sensor Fusion Technology**
- **Hardware-Software Bridge**

</td>
<td width="50%">

### EDUCATIONAL POWER
- **Algorithm Comparison**
- **Performance Analytics**
- **Research Foundation**
- **Industry Applications**

</td>
</tr>
</table>

---

## 🎯 USAGE FLOW

```mermaid
sequenceDiagram
    participant E as ESP32
    participant W as Webots
    participant V as Visualization
    
    E->>W: WiFi Connection
    W->>E: Robot Status
    E->>E: Path Planning
    E->>W: Movement Commands
    W->>V: Live Updates
    V->>V: Real-time Display
    
    Note over E,V: Continuous Loop Until Goal Reached
```

---

<div align="center">

## POWER STATISTICS

```ascii
╔═══════════════════════════════════════════════════════════╗
║  🔮 ALGORITHMS: 3 Advanced Types  │  ⚡ PLANNING: Real-time  ║
║  🧠 ESP32 Brain: MicroPython      │  📡 COMMUNICATION: WiFi  ║  
║  🎯 NAVIGATION: Grid-based        │  📊 VISUALIZATION: Live  ║
╚═══════════════════════════════════════════════════════════╝
```

### LICENSE: MIT | CONTRIBUTE: GitHub | WATCH: Demo Above

**Where hardware transcends software limitations, intelligence emerges from silicon and code**

</div> 
