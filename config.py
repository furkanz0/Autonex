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
#  ROUTE / MAP  (Town05 — urban grid)
# =====================================================================
WANT_START = carla.Location(x=-100.0, y=30.0, z=0.0)
WANT_END   = carla.Location(x=  80.0, y=-140.0, z=0.0)

GOAL_M = 20.0
MAX_S  = 800

# =====================================================================
#  SPECTATOR CAMERA
# =====================================================================
CAM_BACK = -9.0
CAM_UP   =  6.0
CAM_PIT  = -18.0

# =====================================================================
#  MULTI-WINDOW LAYOUT  (1920×1080 screen)
# =====================================================================
#  ┌──────────┐ ┌──────────────────┐ ┌──────────────────┐
#  │ MiniMap  │ │    DroneCam      │ │    ChaseCam      │
#  │ 320×320  │ │    780×500       │ │    780×500       │
#  │ (left)   │ │    (center)      │ │    (right)       │
#  └──────────┘ └──────────────────┘ └──────────────────┘

SCREEN_W, SCREEN_H = 1920, 1080

# MiniMap (OpenCV) — far left, vertically centered
MINIMAP_SIZE = 320
MINIMAP_X    = 10
MINIMAP_Y    = (SCREEN_H - MINIMAP_SIZE) // 2  # 380

# DroneCam (Pygame) — center
W, H    = 780, 500
DRONE_X = 340
DRONE_Y = (SCREEN_H - H) // 2  # 290

# ChaseCam (OpenCV) — right
CHASE_W, CHASE_H = 780, 500
CHASE_X = 1130
CHASE_Y = (SCREEN_H - CHASE_H) // 2  # 290

# DroneCam sensor params
DFOV = 90
DH   = 22.0
DB   = -14.0
DP   = -32.0

# ChaseCam sensor params (third-person chase)
CHASE_FOV  = 90
CHASE_BACK = -9.0   # behind the vehicle
CHASE_UP   = 6.0    # above the vehicle
CHASE_PIT  = -18.0  # pitch angle

# =====================================================================
#  GREEN LINE
# =====================================================================
GL_N    = 60      # number of points
GL_STEP = 3.5     # spacing (m)
GL_LIFE = 0.11    # lifetime
GL_SZ   = 0.12
GL_COL  = carla.Color(0, 150, 255)

# =====================================================================
#  PID / CONTROL CONSTANTS  (Waypoint Mode)
# =====================================================================
KP_STEER        = 0.9      # proportional steer
LOOKAHEAD_M     = 8.0      # lookahead distance (m)
THROTTLE_CRUISE = 0.55     # cruise throttle (adjusted by speed)
BRAKE_KP        = 0.5      # proportional brake (on overspeed)

# =====================================================================
#  LANE DETECTION  (--lane mode)
# =====================================================================
# — Ön kamera sensörü —
LANE_CAM_W, LANE_CAM_H = 640, 480   # Ön kamera çözünürlüğü
LANE_CAM_FOV = 110                    # Geniş açı (şerit kenarları)
LANE_CAM_X   = 2.2                    # Sensör konumu: aracın önü (m)
LANE_CAM_Z   = 1.4                    # Sensör yüksekliği (m)
LANE_CAM_PIT = -8.0                   # Hafif aşağı bakış açısı (°)

# — HLS renk uzayı eşikleri —
#   Beyaz şerit: yüksek Lightness, düşük Saturation
WHITE_L_THRESH = (190, 255)
WHITE_S_THRESH = (0, 60)
#   Sarı şerit: belirli Hue aralığı + orta Saturation
YELLOW_H_THRESH = (15, 35)
YELLOW_S_THRESH = (80, 255)
YELLOW_L_THRESH = (120, 255)

# — Canny kenar tespiti —
CANNY_LOW  = 50
CANNY_HIGH = 150

# — Perspective Warp (kuşbakışı dönüşüm) —
#   Kaynak noktalar (640×480 orijinal görüntü üzerinde trapez)
WARP_SRC = [(180, 310), (460, 310), (20, 470), (620, 470)]
#   Hedef noktalar (kuşbakışı dikdörtgen)
WARP_DST = [(120, 0), (520, 0), (120, 480), (520, 480)]

# — Sliding window (kayan pencere) —
SW_NWINDOWS = 12           # Dikey pencere sayısı
SW_MARGIN   = 60           # Pencere yarı genişliği (piksel)
SW_MINPIX   = 40           # Yeni merkez için minimum piksel

# — Lane PID kontrolcüsü —
LANE_KP = 0.007            # Oransal (Proportional)
LANE_KI = 0.0001           # İntegral
LANE_KD = 0.004            # Türev (Derivative)

# — Lane mode hız / gaz —
LANE_SPEED_KMH  = 25.0     # Hedef hız (düşük = güvenli)
LANE_SPEED_MS   = LANE_SPEED_KMH / 3.6
LANE_THROTTLE   = 0.45     # Cruise gaz
LANE_BRAKE_KP   = 0.5      # Aşırı hızda fren

# — Kavşak / şerit kaybı fallback —
LANE_LOST_MAX   = 20       # Bu kadar frame şerit yoksa waypoint'e geç
LANE_MIN_PIXELS = 800      # Minimum şerit pikseli (altıysa "kayıp")

# — Debug overlay penceresi —
LANE_DEBUG_WIN  = True      # Debug penceresi aç (overlay + warped)
LANE_DEBUG_W    = 480       # Debug pencere genişliği
LANE_DEBUG_H    = 360       # Debug pencere yüksekliği

