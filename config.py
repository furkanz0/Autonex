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
#  ROUTE / MAP  (Town10HD — modern urban city)
# =====================================================================
WANT_START = carla.Location(x=-50.0, y=25.0, z=0.0)
WANT_END   = carla.Location(x= 90.0, y=-55.0, z=0.0)

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
#  LANE FOLLOWING  (--lane mode)
# =====================================================================

# — Ön kamera sensörü —
LANE_CAM_W, LANE_CAM_H = 640, 480   # Ön kamera çözünürlüğü
LANE_CAM_FOV = 110                    # Geniş açı (şerit kenarları)
LANE_CAM_X   = 2.2                    # Sensör konumu: aracın önü (m)
LANE_CAM_Z   = 1.4                    # Sensör yüksekliği (m)
LANE_CAM_PIT = -8.0                   # Hafif aşağı bakış açısı (°)

# — HLS renk uzayı eşikleri (LaneDetector) —
WHITE_L_THRESH = (190, 255)
WHITE_S_THRESH = (0, 60)
YELLOW_H_THRESH = (15, 35)
YELLOW_S_THRESH = (80, 255)
YELLOW_L_THRESH = (120, 255)

# — Canny kenar tespiti —
CANNY_LOW  = 50
CANNY_HIGH = 150

# — Perspective Warp (kuşbakışı dönüşüm) —
WARP_SRC = [(180, 310), (460, 310), (20, 470), (620, 470)]
WARP_DST = [(120, 0), (520, 0), (120, 480), (520, 480)]

# — Sliding window (kayan pencere) —
SW_NWINDOWS = 12
SW_MARGIN   = 60
SW_MINPIX   = 40

# — Piksel → Metre dönüşüm faktörleri (LaneDetector) —
LANE_XM_PER_PIX = 3.7 / 400    # x ekseni: 3.7m şerit ≈ 400px (warped)
LANE_YM_PER_PIX = 30.0 / 480   # y ekseni: 30m görüş ≈ 480px (warped)

# — Lane PID kontrolcüsü (Vision Mode — metre bazlı offset) —
LANE_KP = 0.15                  # Oransal (P) - Savrulmayı (overshoot) önlemek için düşürüldü
LANE_KI = 0.001                 # İntegral (I) - Yavaşça toparlama
LANE_KD = 0.30                  # Türev (D) - Titreşimi (oscillation) sönümlemek için artırıldı
LANE_HEADING_KP = 0.5           # Heading düzeltme katsayısı (compute_map)

# — Lookahead mesafeleri —
LANE_LOOKAHEAD_M = 8.0          # Normal seyir lookahead (m)
LANE_CHANGE_LOOKAHEAD_M = 12.0  # Şerit değiştirme lookahead (m)

# — Hız kontrolü —
LANE_CRUISE_KMH = 30.0          # Seyir hızı (km/h)
LANE_CURVE_KMH  = 20.0          # Viraj hızı (km/h)
LANE_CURVE_RADIUS_THR = 100.0   # Viraj eşik yarıçapı (m)

# — Şerit değiştirme parametreleri —
LANE_CHANGE_KMH = 25.0              # Şerit değiştirme hedef hızı
LANE_CHANGE_TARGET_M = 3.5          # Hedef şerit offset mesafesi (m)
LANE_CHANGE_CENTER_TOL_M = 0.4      # Merkez toleransı (m)
LANE_CHANGE_STABLE_FRAMES = 15      # Stabil sayılma frame sayısı
LANE_CHANGE_SETTLE_FRAMES = 40      # Yerleşme frame sayısı
LANE_CHANGE_RAMP_FRAMES = 30        # Rampa frame sayısı
LANE_CHANGE_MIN_FRAMES = 20         # Minimum şerit değiştirme frame
LANE_CHANGE_MAX_FRAMES = 200        # Maksimum şerit değiştirme frame
LANE_CHANGE_COMPLETE_RATIO = 0.7    # Tamamlanma oranı
LANE_MAX_STEER_DELTA = 0.015        # Frame başına maks direksiyon değişimi

# =====================================================================
#  TRAFFIC LIGHT DETECTION (Kamera — HSV Renk Tespiti)
# =====================================================================
# — Kırmızı ışık HSV aralıkları (kırmızı 0/180'de sarmalanır) —
TL_RED_HSV_LOW1  = (0,   120, 120)    # Alt aralık: H=0-10
TL_RED_HSV_HIGH1 = (10,  255, 255)
TL_RED_HSV_LOW2  = (170, 120, 120)    # Üst aralık: H=170-180
TL_RED_HSV_HIGH2 = (180, 255, 255)

# — Yeşil ışık HSV aralığı —
TL_GREEN_HSV_LOW  = (40, 80, 80)
TL_GREEN_HSV_HIGH = (90, 255, 255)

# — Kontur filtreleri —
TL_MIN_AREA = 60                  # Minimum kontur alanı (piksel²)
TL_MAX_AREA = 12000               # Maksimum kontur alanı
TL_MIN_CIRCULARITY = 0.3          # Minimum dairesellik (0-1)

# — Algılama bölgesi ve doğrulama —
TL_ROI_RATIO = 0.55               # Görüntünün üst %55'i (ışıklar yukarıda)
TL_CONFIRM_FRAMES = 3             # Tepki için ardışık tespit frame sayısı
TL_BRAKE_AREA_SCALE = 2000.0      # Fren yoğunluğu alan ölçeği

