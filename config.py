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
GL_COL  = carla.Color(0, 150, 255)

# =====================================================================
#  PID / CONTROL CONSTANTS
# =====================================================================
KP_STEER        = 0.9      # proportional steer
LOOKAHEAD_M     = 8.0      # lookahead distance (m)
THROTTLE_CRUISE = 0.55     # cruise throttle (adjusted by speed)
BRAKE_KP        = 0.5      # proportional brake (on overspeed)


