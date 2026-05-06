"""
╔══════════════════════════════════════════════════════════════════════╗
║  config.py — All constants and settings                             ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import sys
import os

# ── CARLA Python API path ─────────────────────────────────────────────
CARLA_AGENTS = r"C:\Users\Furkan\Desktop\Carla\PythonAPI\carla"
if os.path.exists(CARLA_AGENTS):
    sys.path.insert(0, CARLA_AGENTS)

try:
    import carla
except ImportError:
    print("[ERROR] carla not found")
    sys.exit(1)

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: F401
    HAS_GRP = True
except ImportError:
    HAS_GRP = False
    print("[WARNING] GlobalRoutePlanner not available, using simple .next() chain")

try:
    import pygame  # noqa: F401
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False
    print("[WARNING] pygame not found: pip install pygame")


# =====================================================================
#  CONNECTION
# =====================================================================
HOST        = "localhost"
PORT        = 2000
TIMEOUT_S   = 20.0
FIXED_DELTA = 0.05

# =====================================================================
#  SPEED
# =====================================================================
TARGET_SPEED_KMH = 30.0
TARGET_SPEED_MS  = TARGET_SPEED_KMH / 3.6

# =====================================================================
#  ROUTE / MAP
# =====================================================================
WANT_START = carla.Location(x=-200.0, y=14.0, z=0.0)
WANT_END   = carla.Location(x= 300.0, y=14.0, z=0.0)

GOAL_M = 20.0
MAX_S  = 500

# =====================================================================
#  SPECTATOR CAMERA
# =====================================================================
CAM_BACK = -9.0
CAM_UP   =  6.0
CAM_PIT  = -18.0

# =====================================================================
#  PYGAME DRONE CAMERA
# =====================================================================
W, H = 960, 540
DFOV = 90
DH   = 22.0
DB   = -14.0
DP   = -32.0

# =====================================================================
#  GREEN LINE
# =====================================================================
GL_N    = 60      # number of points
GL_STEP = 3.5     # spacing (m)
GL_LIFE = 0.11    # lifetime
GL_SZ   = 0.12
GL_COL  = carla.Color(0, 225, 65)

# =====================================================================
#  PID / CONTROL CONSTANTS
# =====================================================================
KP_STEER        = 0.9      # proportional steer
LOOKAHEAD_M     = 8.0      # lookahead distance (m)
THROTTLE_CRUISE = 0.55     # cruise throttle (adjusted by speed)
BRAKE_KP        = 0.5      # proportional brake (on overspeed)

# =====================================================================
#  LANE DETECTION — CAMERA
# =====================================================================
LANE_CAM_W     = 960
LANE_CAM_H     = 540
LANE_CAM_FOV   = 90         # reduced from 110 for less lens distortion
LANE_CAM_X     = 2.0        # forward offset from vehicle center (m)
LANE_CAM_Z     = 1.4        # height (m)
LANE_CAM_PITCH = -8.0       # slightly more downward for better lane view

# =====================================================================
#  LANE DETECTION — PERSPECTIVE TRANSFORM (normalized 0-1)
# =====================================================================
LANE_SRC = [(0.15, 0.90), (0.85, 0.90), (0.60, 0.60), (0.40, 0.60)]
LANE_DST = [(0.20, 1.00), (0.80, 1.00), (0.80, 0.00), (0.20, 0.00)]

# =====================================================================
#  LANE DETECTION — SLIDING WINDOW
# =====================================================================
LANE_NWINDOWS  = 12        # number of sliding windows
LANE_MARGIN    = 100        # half-width of each window (px) — wider for tolerance
LANE_MINPIX    = 30        # min pixels to recenter window

# =====================================================================
#  LANE DETECTION — COLOR THRESHOLDS
# =====================================================================
LANE_WHITE_L_THRESH  = 220          # HLS L-channel min for white (increased to ignore grey road)
LANE_YELLOW_LOWER    = (15, 60, 100)   # HSV lower bound (relaxed)
LANE_YELLOW_UPPER    = (40, 255, 255)  # HSV upper bound
LANE_SOBEL_LOW       = 25
LANE_SOBEL_HIGH      = 200
LANE_S_THRESH_LOW    = 80
LANE_S_THRESH_HIGH   = 255

# =====================================================================
#  LANE FOLLOWING — PID
# =====================================================================
LANE_KP = 0.40
LANE_KI = 0.0003
LANE_KD = 0.05

# =====================================================================
#  LANE FOLLOWING — SPEED
# =====================================================================
LANE_CRUISE_KMH       = 20.0     # straight road speed (safer)
LANE_CURVE_KMH        = 12.0     # tight curve speed
LANE_CURVE_RADIUS_THR = 200.0    # curvature threshold (m)
LANE_HISTORY_LEN      = 5        # frames to average for smoothing
