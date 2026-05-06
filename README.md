# Autonex: Vision-Based Autonomous Driving Simulation 🚗👁️

![Python](https://img.shields.io/badge/Python-3.12-blue.svg)
![CARLA](https://img.shields.io/badge/CARLA-0.9.16-orange.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-green.svg)

## 📌 Simulation Overview
**Autonex** is an advanced autonomous driving simulation developed using Python and the CARLA Simulator. The core objective of the project is to navigate a virtual vehicle safely and legally through complex urban environments by relying entirely on real-time computer vision and image processing techniques.

## 🚀 Core Features
*   **Real-Time Lane Tracking:** Processes continuous visual data from onboard RGB cameras to detect lane markings, calculating the necessary steering adjustments to keep the vehicle strictly centered in its lane.
*   **Traffic Sign & Light Recognition:** Actively scans the environment for traffic lights and speed limit signs using color masking and contour detection, ensuring full compliance with urban traffic regulations.
*   **Dynamic Obstacle Avoidance:** Monitors the ego-vehicle's path for dynamic obstacles, including pedestrians and other vehicles, triggering safety protocols (braking/stopping) when necessary.

## 🛠️ System Architecture
The system is divided into three main pipelines to ensure modularity and efficiency:
1.  **Global Planner (Environment & Routing):** Manages the CARLA server connection, ego-vehicle spawning, and waypoint generation.
2.  **Perception (Computer Vision):** Captures RGB sensor data and applies OpenCV algorithms (Grayscale, Gaussian Blur, Canny Edge Detection, Hough Transform) to extract environmental features.
3.  **Control & Decision (FSM & PID):** A Finite State Machine evaluates perception data to make driving decisions, while a PID controller outputs smooth steering, throttle, and brake commands.

## 💻 Tech Stack
*   **Simulation Engine:** CARLA Simulator
*   **Programming Language:** Python
*   **Computer Vision:** OpenCV, NumPy

## 👥 Team    
Developed by the Software Engineering project team at Erciyes University.
*   Furkan Zorlu
*   Erdem Develioğlu
*   Abdullah Karaismailoğlu
