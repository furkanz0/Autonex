<p align="center">
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/CARLA-0.9.16-FF6F00?style=for-the-badge&logo=unrealengine&logoColor=white" />
  <img src="https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white" />
  <img src="https://img.shields.io/badge/Pygame-2.x-00D162?style=for-the-badge&logo=pygame&logoColor=white" />
  <img src="https://img.shields.io/badge/NumPy-Scientific-013243?style=for-the-badge&logo=numpy&logoColor=white" />
</p>

<h1 align="center">🚗 Autonex</h1>
<h3 align="center">Vision-Based Autonomous Driving Simulation</h3>

<p align="center">
  <i>A real-time autonomous driving simulation powered entirely by Computer Vision.</i><br/>
  <i>Lane following, traffic light recognition, and adaptive cruise control on CARLA Simulator.</i>
</p>

<br/>

<p align="center">
  <img src="screenshots/simulation_overview.png" width="90%" alt="Simulation Overview"/>
  <br/><i>Full simulation view — MiniMap, Lane Camera, Traffic Light Detector, and Vehicle Detection panels running simultaneously</i>
</p>

---

## 📌 About the Project

**Autonex** is an advanced autonomous driving simulation developed using CARLA Simulator and Python. The primary goal is to navigate a vehicle through complex urban environments **entirely using real-time computer vision and image processing techniques** in a safe and traffic-compliant manner.

> **No deep learning models or pre-trained networks are used.**  
> All perception is performed using classical OpenCV algorithms (HSV color space, Canny edge detection, Hough transform, contour analysis, perspective warp).

---

## 🚀 Key Features

### 🛣️ Real-Time Lane Following
Processing pipeline applied to front RGB camera images:

| Step | Technique | Description |
|:----:|-----------|-------------|
| 1 | **HLS Color Filter** | Isolation of white and yellow lane markings |
| 2 | **Canny Edge Detection** | Edge detection for lane boundary identification |
| 3 | **Perspective Warp** | Transformation to bird's-eye view |
| 4 | **Sliding Window** | Histogram-based sliding window for lane pixel detection |
| 5 | **2nd Degree Polynomial Fit** | Mathematical modeling of lane curvature |
| 6 | **PID Control** | Steering control based on lateral offset and curvature |

<p align="center">
  <img src="screenshots/lane_detection.png" width="70%" alt="Lane Detection Pipeline"/>
  <br/><i>Lane Dashboard — Camera overlay + bird's-eye warped view for lane detection</i>
</p>

### 🚦 Traffic Light Recognition
Camera-based detection using pure OpenCV HSV color analysis:

- **Dual HSV Range:** Dual mask for red color wrapping at 0°/180°
- **Contour Analysis:** Area, circularity, aspect ratio, and extent filters
- **Dark Body Verification:** Traffic light housing validation (false positive reduction)
- **Multi-Frame Confirmation:** Detection in 3+ consecutive frames → confirmed stop decision
- **CARLA World Validation:** Camera detection + world data cross-validation

<p align="center">
  <img src="screenshots/traffic_light.png" width="70%" alt="Traffic Light Detection"/>
  <br/><i>Traffic Light Detector — HSV color masks and contour analysis for traffic light detection</i>
</p>

### 🚙 OpenCV Vehicle Detection & Adaptive Cruise Control (ACC)
Detects other vehicles from the front camera image and estimates distance:

- **Canny + Background Subtraction** for moving object detection
- **Perspective geometry** for distance estimation from pixel width (in meters)
- **4-state FSM:** `FREE_DRIVE → FOLLOWING → BRAKING → EMERGENCY`
- **EMA smoothing** to prevent abrupt throttle/brake transitions

<p align="center">
  <img src="screenshots/vehicle_detection.png" width="70%" alt="Vehicle Detection & ACC"/>
  <br/><i>Vehicle Detection — Bounding box vehicle detection and ACC distance control</i>
</p>

### 🗺️ Interactive Map Navigation
Pygame-based bird's-eye map navigator:

- **Left Click** → Set start point (snaps to road)
- **Right Click** → Set end point (snaps to road)
- **Scroll** → Zoom in / out
- **Middle Click Drag** → Pan the map
- **ENTER** → Calculate route and start simulation
- **GlobalRoutePlanner** for automatic route generation

<p align="center">
  <img src="screenshots/map_navigator.png" width="70%" alt="Map Navigator"/>
  <br/><i>Interactive Map Navigator — Route selection on Town10HD</i>
</p>

### 🔧 Additional Features

| Feature | Description |
|---------|-------------|
| **NPC Traffic** | Dense traffic environment with 100 autonomous NPC vehicles |
| **Lane Changing** | Manual lane change with `A/D` or `←/→` keys |
| **Weather Control** | Switch between sunny / rainy / snowy with `1/2/3` keys |
| **Multi-Camera** | Simultaneous DroneCam, ChaseCam, and MiniMap display |
| **Stall Recovery** | Automatic recovery mechanism after 5 seconds of inactivity |
| **Green Route Line** | Real-time waypoint route visualization on the world |

---

## 🛠️ System Architecture

The project uses a three-layer modular architecture based on the **MVC (Model-View-Controller)** design pattern:

```
┌─────────────────────────────────────────────────────────────────────┐
│                          main.py (Entry Point)                      │
│             Mode selection: --map / --lane / default                │
└────────┬──────────────────────┬──────────────────────┬──────────────┘
         │                      │                      │
    ┌────▼──────┐         ┌─────▼─────┐         ┌─────▼──────┐
    │  MODELS   │         │   VIEWS   │         │ CONTROLLERS│
    │(Perception│         │ (Display) │         │ (Control)  │
    └───────────┘         └───────────┘         └────────────┘

 ╔═══════════════╗    ╔════════════════════╗    ╔═══════════════════╗
 ║  lane_detector ║    ║  lane_dashboard    ║    ║  simulation       ║
 ║  vehicle_det.  ║    ║  traffic_light_p.  ║    ║  lane_controller  ║
 ║  route         ║    ║  vehicle_det_panel ║    ║  traffic_light_c. ║
 ║  connection    ║    ║  map_navigator     ║    ║  acc_controller   ║
 ║  vehicle       ║    ║  minimap           ║    ║  traffic_rules_e. ║
 ║  traffic       ║    ║  chase_cam         ║    ║  vehicle_ctrl.    ║
 ║  npc_manager   ║    ║  drone_cam         ║    ╚═══════════════════╝
 ╚═══════════════╝    ║  lane_camera       ║
                      ║  spectator         ║
                      ╚════════════════════╝
```

### Perception Pipelines

```
Front Camera (640×480, FOV 110°)
       │
       ├──▶ LaneDetector ──▶ HLS → Canny → Warp → Sliding Window → Polynomial Fit
       │                         └──▶ lateral_offset_m, curvature_m, confidence
       │
       ├──▶ TrafficLightDetector ──▶ HSV Masking → Contour → Circularity Filter
       │                                └──▶ state (red/green/none), should_stop
       │
       └──▶ VehicleDetector ──▶ Canny + MOG2 → Contour → Perspective Distance
                                    └──▶ closest_distance_m, vehicle_count
                                              │
                                    ┌─────────▼──────────┐
                                    │  TrafficRulesEngine │
                                    │ (Decision Merging)  │
                                    └─────────┬──────────┘
                                              │
                                    ┌─────────▼──────────┐
                                    │   AccController    │
                                    │  (FSM: FREE→EMERG) │
                                    └────────────────────┘
```

---

## 📂 Project Structure

```
Autonex/
├── main.py                          # Main entry point and mode management
├── config.py                        # All constants and parameters
│
├── models/                          # 🧠 Perception & Data Layer
│   ├── lane_detector.py             #    Lane detection pipeline (OpenCV)
│   ├── vehicle_detector.py          #    Vehicle detection & distance estimation
│   ├── route.py                     #    Route planning (GlobalRoutePlanner)
│   ├── connection.py                #    CARLA server connection management
│   ├── vehicle.py                   #    Ego vehicle spawn & physics
│   ├── traffic.py                   #    NPC traffic management
│   └── npc_manager.py              #    NPC vehicle lifecycle
│
├── views/                           # 🖥️ Visualization Layer
│   ├── lane_dashboard.py            #    Lane detection dashboard panel
│   ├── traffic_light_panel.py       #    Traffic light debug panel
│   ├── vehicle_detection_panel.py   #    Vehicle detection overlay panel
│   ├── map_navigator.py             #    Interactive map navigator
│   ├── minimap.py                   #    Real-time minimap
│   ├── chase_cam.py                 #    3rd person chase camera
│   ├── drone_cam.py                 #    Bird's-eye drone camera
│   ├── lane_camera.py               #    Front camera sensor management
│   ├── lane_cam.py                  #    Lane camera window
│   ├── spectator.py                 #    CARLA spectator control
│   └── green_line.py                #    Route line drawing
│
├── controllers/                     # 🎮 Control & Decision Layer
│   ├── simulation.py                #    Main simulation loop (orchestrator)
│   ├── lane_controller.py           #    PID lane following controller
│   ├── traffic_light_controller.py  #    Camera traffic light detection
│   ├── acc_controller.py            #    Adaptive Cruise Control (ACC)
│   ├── traffic_rules_engine.py      #    Traffic rules decision engine
│   └── vehicle_controller.py        #    Waypoint PID controller
│
└── utils/                           # 🔧 Utilities
    └── logger.py                    #    Formatted console logger
```

---

## ⚙️ Running Modes

```bash
# 1. Default Route — Fixed start/end points with waypoint PID
python main.py

# 2. Map Navigator — Interactive route selection
python main.py --map

# 3. Lane Following — Camera-based autonomous driving
python main.py --lane

# 4. Map + Lane — Interactive route selection + camera lane following
python main.py --map --lane
```

### Keyboard Controls (In-Simulation)

| Key | Function |
|:---:|----------|
| `A` / `←` | Change lane left |
| `D` / `→` | Change lane right |
| `1` | ☀️ Sunny weather |
| `2` | 🌧️ Rainy weather |
| `3` | ❄️ Snowy weather |
| `R` | 🔴 Force traffic light to red |
| `G` | 🟢 Force traffic light to green |

---

## 🖥️ Multi-Window Layout

```
┌──────────┐ ┌──────────────────┐ ┌──────────────────┐
│ MiniMap  │ │    DroneCam      │ │    ChaseCam      │
│ 320×320  │ │    780×500       │ │    780×500       │
│          │ │                  │ │                  │
└──────────┘ └──────────────────┘ └──────────────────┘
┌──────────────────────────┐ ┌──────────────────────────┐
│   Traffic Light Panel    │ │  Vehicle Detection Panel │
│                          │ │                          │
└──────────────────────────┘ └──────────────────────────┘
              ┌──────────────────────────┐
              │     Lane Dashboard       │
              │  (Warped + Overlay)      │
              └──────────────────────────┘
```

---

## 💻 Tech Stack

| Category | Technology | Purpose |
|----------|-----------|---------|
| **Simulation** | CARLA 0.9.16 | Realistic urban driving environment (Town10HD) |
| **Programming** | Python 3.12 | All application logic |
| **Image Processing** | OpenCV 4.x | Lane, traffic light, and vehicle detection |
| **Scientific Computing** | NumPy | Matrix operations, polynomial fit, perspective transform |
| **GUI / Map** | Pygame | Interactive map navigator and camera windows |

---

## 🔧 Installation

### Requirements
- CARLA Simulator 0.9.16
- Python 3.12+
- The following Python packages:

```bash
pip install opencv-python numpy pygame
```

### CARLA Connection
Update the CARLA PythonAPI path in the `config.py` file:

```python
CARLA_AGENTS = r"C:\<CARLA_INSTALL_PATH>\PythonAPI\carla"
```

### Running
1. Start the CARLA Simulator
2. Run one of the following commands:

```bash
python main.py --map --lane
```

---

## 📸 Screenshots

<p align="center">
  <img src="screenshots/map_navigator.png" width="90%" alt="Map Navigator"/>
  <br/><i>🗺️ Interactive Map Navigator — Start and end point selection on the Town10HD map with automatic route generation</i>
</p>

<p align="center">
  <img src="screenshots/simulation_overview.png" width="90%" alt="Simulation Overview"/>
  <br/><i>🚗 Full Simulation View — MiniMap, Lane Camera, Traffic Light Detector, and Vehicle Detection panels running simultaneously</i>
</p>

<p align="center">
  <img src="screenshots/chase_cam.jpg" width="90%" alt="Chase Camera View"/>
  <br/><i>🎥 ChaseCam View — 3rd person chase camera with autonomous driving and route line</i>
</p>

---

## 👥 Team

Developed by the Software Engineering project team at Erciyes University.

| Name | Role |
|------|------|
| **Furkan Zorlu** | Developer |
| **Erdem Develioğlu** | Developer |
| **Abdullah Karaismailoğlu** | Developer |

---

<p align="center">
  <sub>Erciyes University — Software Engineering — 2026</sub>
</p>
