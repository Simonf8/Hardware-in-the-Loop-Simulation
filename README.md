<div align="center">

# HARDWARE-IN-THE-LOOP NAVIGATION SYSTEM
    PLEASE DON'T COPY EVERYTHING AND HAND IT IN AS YOUR WORK, USE IT AS INSPIRATION

<img src="https://img.shields.io/badge/STATUS-ACTIVE-00E5FF?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjAiIGhlaWdodD0iMjAiIHZpZXdCb3g9IjAgMCAyMCAyMCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPGNpcmNsZSBjeD0iMTAiIGN5PSIxMCIgcj0iOCIgZmlsbD0iIzAwRTVGRiIvPgo8L3N2Zz4K"/>
<img src="https://img.shields.io/badge/ESP32-MICROPYTHON-4A148C?style=for-the-badge"/>
<img src="https://img.shields.io/badge/WEBOTS-R2023A+-6A0DAD?style=for-the-badge"/>
<img src="https://img.shields.io/badge/ALGORITHMS-3_TYPES-E91E63?style=for-the-badge"/>

```ascii
╔══════════════════════════════════════════════════════════════╗
║                     REAL HARDWARE CONTROLS VIRTUAL ROBOT                    ║
║                                                              ║
║  ESP32 ←→ WiFi ←→ Webots  |  Live Pathfinding  |  Real-time HIL  ║
╚══════════════════════════════════════════════════════════════╝
```

## LIVE DEMONSTRATION

[![Demo](https://img.shields.io/badge/▶_WATCH_LIVE_DEMO-DIJKSTRA_NAVIGATION-2D1B69?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/QRNA4Zo2aBg)

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
