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
#  PERFORMANCE / PRESENTATION
# =====================================================================
# These keep the driving/control loop at FIXED_DELTA while reducing the
# cost of debug visuals and OpenCV window refreshes.
LANE_DASHBOARD_RENDER_EVERY = 2      # 2 = 10 FPS UI at 20 FPS sim
TL_PANEL_RENDER_EVERY       = 3      # traffic-light presentation panel
MINIMAP_RENDER_EVERY        = 4
CARLA_LANE_DEBUG_EVERY      = 1      # 0 disables CARLA in-world lane debug

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
GL_COL  = carla.Color(90, 150, 200)

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
TL_MIN_BBOX_PX = 6                # Minimum lamp bbox size
TL_MAX_BBOX_RATIO = 0.16          # Max lamp bbox size relative to frame
TL_MIN_ASPECT_RATIO = 0.45        # Reject long/thin colored objects
TL_MAX_ASPECT_RATIO = 1.8
TL_MIN_EXTENT = 0.35              # Contour fill ratio inside bbox
TL_MAX_EXTENT = 0.92
TL_MIN_DARK_CONTEXT = 0.18        # Require dark traffic-light housing around lamp
TL_CONTEXT_PAD_RATIO = 1.7        # Context expansion around the colored lamp
TL_CENTER_MARGIN_RATIO = 0.08     # Ignore far-edge color clutter

# — Algılama bölgesi ve doğrulama —
TL_ROI_RATIO = 0.55               # Görüntünün üst %55'i (ışıklar yukarıda)
TL_CONFIRM_FRAMES = 3             # Tepki için ardışık tespit frame sayısı
TL_BRAKE_AREA_SCALE = 2000.0      # Fren yoğunluğu alan ölçeği
TL_WORLD_VALIDATE_DISTANCE_M = 35.0
TL_WORLD_VALIDATE_LATERAL_M = 12.0

# =====================================================================
#  NPC TRAFFIC
# =====================================================================
NPC_TRAFFIC_ENABLED = True
NPC_TRAFFIC_COUNT = 100            # Dense traffic; may affect performance
NPC_TRAFFIC_SEED = 42
NPC_MIN_EGO_SPAWN_DISTANCE_M = 35.0
NPC_SPEED_DIFF_PERCENT = 35.0      # Positive means NPCs drive slower
NPC_FOLLOW_DISTANCE_M = 7.0
NPC_SPAWN_SETTLE_TICKS = 8

# Pedestrian NPCs
NPC_WALKERS_ENABLED = True
NPC_WALKER_COUNT = 36
NPC_WALKER_MIN_EGO_SPAWN_DISTANCE_M = 5.0
NPC_WALKER_SPAWN_RADIUS_M = 120.0
NPC_WALKER_AHEAD_DISTANCE_M = 95.0
NPC_WALKER_SIDE_DISTANCE_M = 35.0
NPC_WALKER_CROSSWALK_RATIO = 0.25
NPC_WALKER_FORCED_CROSSING_COUNT = 0
NPC_WALKER_RUNNING_PERCENT = 0.0
NPC_WALKER_CROSSING_FACTOR = 0.0

# Extra safety for locally controlled waypoint/PID driving.
LEAD_VEHICLE_SLOW_DISTANCE_M = 24.0
LEAD_VEHICLE_STOP_DISTANCE_M = 11.0
LEAD_VEHICLE_LATERAL_MARGIN_M = 2.4


# =====================================================================
#  NPC ARAÇ YÖNETİMİ
# =====================================================================
NPC_COUNT            = 8          # Başlangıçta spawn edilecek NPC sayısı
NPC_SPAWN_RADIUS_M   = 80.0       # Ego araç çevresinde spawn alanı (m)
NPC_MIN_DIST_M       = 20.0       # Minimum spawn mesafesi (ego'dan)
NPC_TARGET_SPEED_KMH = 25.0       # NPC hedef hızı (km/h)
NPC_SAFE_DISTANCE_M  = 10.0       # Traffic Manager güvenli takip mesafesi (m)
NPC_MODELS = [                    # Kullanılacak araç modelleri (çeşitlilik)
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.bmw.grandtourer",
    "vehicle.citroen.c3",
    "vehicle.dodge.charger_2020",
    "vehicle.lincoln.mkz_2020",
    "vehicle.mercedes.coupe",
    "vehicle.nissan.micra",
    "vehicle.seat.leon",
    "vehicle.toyota.prius",
    "vehicle.volkswagen.t2",
]


# =====================================================================
#  OPENCV ARAÇ TESPİTİ (Vehicle Detection)
# =====================================================================
VD_ROI_TOP_RATIO     = 0.30       # Araç ROI başlangıcı: görüntünün üst %30'u atla
VD_MIN_AREA          = 2000       # Minimum kontur alanı (piksel²)
VD_MAX_AREA          = 90000      # Maksimum kontur alanı
VD_MIN_ASPECT        = 0.8        # Min genişlik/yükseklik oranı (araç şekli)
VD_MAX_ASPECT        = 2.5        # Max genişlik/yükseklik oranı
VD_REAL_WIDTH_M      = 1.8        # Ortalama araç genişliği (metre)
VD_FOCAL_LENGTH_PX   = 554        # 640px genişlik @ 90° FOV focal length
VD_SAFE_DIST_M       = 18.0       # Güvenli takip mesafesi — ACC yavaşlama başlar
VD_CRITICAL_DIST_M   = 7.0        # Kritik mesafe — acil fren
VD_EMERGENCY_DIST_M  = 4.0        # Acil durum mesafesi — tam fren
VD_CONFIRM_FRAMES    = 2          # Kaç ardışık frame'de görünmeli (false positive azalt)
VD_MORPH_KERNEL      = 5          # Morfolojik işlem kernel boyutu
VD_CANNY_LOW         = 40         # Canny alt eşiği
VD_CANNY_HIGH        = 120        # Canny üst eşiği
VD_DILATE_ITER       = 2          # Kontur genişletme iterasyon sayısı


# =====================================================================
#  ACC (ADAPTIVE CRUISE CONTROL)
# =====================================================================
ACC_FOLLOW_KP        = 0.04       # Hız takip oransal kazancı
ACC_MAX_THROTTLE     = 0.60       # Maksimum gaz (ACC modunda)
ACC_BRAKE_KP         = 0.08       # Fren oransal kazancı
ACC_MAX_BRAKE        = 0.85       # Maksimum fren (ACC modunda)
ACC_CREEP_THROTTLE   = 0.12       # Yavaş ilerleme gaz değeri

