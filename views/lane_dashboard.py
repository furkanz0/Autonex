"""
views/lane_dashboard.py — Profesyonel şerit takip dashboard penceresi

Koyu temalı, 3 panelli OpenCV arayüzü:
  ┌──────────────────┬──────────────┐
  │  Camera View     │  Bird's-Eye  │
  │  (Lane Overlay)  │  (Warped)    │
  ├──────────────────┴──────────────┤
  │  Telemetry  Dashboard           │
  │  Speed · Offset · Steer · Info  │
  └─────────────────────────────────┘

Klavye:
  A / ← : Sola şerit değiştir
  D / → : Sağa şerit değiştir
  1     : Güneşli hava
  2     : Yağmurlu hava
  3     : Karlı hava
  ESC   : Pencereyi kapat
"""

import cv2
import numpy as np
from models.lane_detector import LaneResult


class LaneDashboard:
    """
    OpenCV tabanlı lane-following dashboard penceresi.

    Kullanım:
        dash = LaneDashboard()
        ...
        alive = dash.render(lane_result, speed, steer, frame_no,
                            lane_state, target_offset)
        cmd = dash.consume_command()    # "left" / "right" / None
        wth = dash.consume_weather()    # "sunny" / "rainy" / "snowy" / None
        ...
        dash.destroy()
    """

    # ── Panel boyutları ──────────────────────────────────────────────
    _CAM_W, _CAM_H = 560, 320
    _WARP_W, _WARP_H = 400, 320
    _DASH_H = 150
    _TOTAL_W = _CAM_W + _WARP_W      # 960
    _TOTAL_H = _CAM_H + _DASH_H      # 470

    # ── Renkler (BGR) ────────────────────────────────────────────────
    _BG      = (25, 25, 35)
    _PANEL   = (35, 35, 50)
    _GREEN   = (0, 230, 118)
    _RED     = (70, 70, 255)
    _CYAN    = (255, 255, 0)
    _YELLOW  = (0, 220, 255)
    _WHITE   = (220, 220, 220)
    _GRAY    = (130, 130, 140)
    _ORANGE  = (30, 160, 255)
    _ACCENT  = (255, 180, 50)
    _DARK    = (18, 18, 28)

    _FONT   = cv2.FONT_HERSHEY_SIMPLEX
    _FONT_S = cv2.FONT_HERSHEY_PLAIN

    # ── Klavye kodları ───────────────────────────────────────────────
    _KEY_ESC   = 27
    _KEY_A     = ord("a")
    _KEY_D     = ord("d")
    _KEY_LEFT  = 0x250000   # OpenCV sol ok
    _KEY_RIGHT = 0x270000   # OpenCV sağ ok
    _KEY_1     = ord("1")
    _KEY_2     = ord("2")
    _KEY_3     = ord("3")
    _KEY_R     = ord("r")
    _KEY_G     = ord("g")

    def __init__(self):
        self._command = None    # bekleyen şerit değiştirme komutu
        self._weather = None    # bekleyen hava durumu komutu
        self._tl_override = None # bekleyen trafik ışığı komutu
        self._window = "Lane Dashboard"

        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, self._TOTAL_W, self._TOTAL_H)

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def render(self, lane: LaneResult, speed_kmh: float, steer: float,
              frame_no: int, lane_change_state: str = "CENTER",
              target_offset: float = 0.0, tl_state: str = "none",
              tl_confirmed: bool = False) -> bool:
        """
        Dashboard'u güncelle ve göster.

        Returns:
            True  = pencere açık, devam et
            False = pencere kapatıldı (ESC veya X)
        """
        canvas = np.full((self._TOTAL_H, self._TOTAL_W, 3),
                         self._BG, dtype=np.uint8)

        # ── Panel 1: Camera Overlay ──────────────────────────────────
        self._draw_camera_panel(canvas, lane)

        # ── Panel 2: Warped View ─────────────────────────────────────
        self._draw_warped_panel(canvas, lane)

        # ── Panel 3: Telemetry Dashboard ─────────────────────────────
        self._draw_telemetry(canvas, lane, speed_kmh, steer, frame_no,
                             lane_change_state, target_offset, tl_state, tl_confirmed)

        # ── Göster ───────────────────────────────────────────────────
        cv2.imshow(self._window, canvas)

        # ── Klavye ───────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFFFFFF

        if key == self._KEY_ESC:
            return False

        if key == self._KEY_A or key == 0x25:   # A veya ←
            self._command = "left"
        elif key == self._KEY_D or key == 0x27:  # D veya →
            self._command = "right"
        elif key == self._KEY_1:
            self._weather = "sunny"
        elif key == self._KEY_2:
            self._weather = "rainy"
        elif key == self._KEY_3:
            self._weather = "snowy"
        elif key == self._KEY_R:
            self._tl_override = "red"
        elif key == self._KEY_G:
            self._tl_override = "green"

        # Pencere kapatıldı mı kontrol et
        try:
            if cv2.getWindowProperty(self._window, cv2.WND_PROP_VISIBLE) < 1:
                return False
        except cv2.error:
            return False

        return True

    def consume_command(self):
        """Bekleyen şerit değiştirme komutunu al ve sıfırla."""
        cmd = self._command
        self._command = None
        return cmd

    def consume_weather(self):
        """Bekleyen hava durumu komutunu al ve sıfırla."""
        wth = self._weather
        self._weather = None
        return wth

    def consume_tl_override(self):
        """Bekleyen trafik ışığı komutunu al ve sıfırla."""
        cmd = self._tl_override
        self._tl_override = None
        return cmd

    def destroy(self):
        """Dashboard penceresini kapat."""
        try:
            cv2.destroyWindow(self._window)
        except Exception:
            pass

    # =================================================================
    #  KAMERA PANELİ
    # =================================================================

    def _draw_camera_panel(self, canvas, lane: LaneResult):
        """Sol üst: Kamera görüntüsü + şerit overlay."""
        if lane.overlay_frame is not None:
            img = cv2.resize(lane.overlay_frame, (self._CAM_W, self._CAM_H))
        else:
            img = np.full((self._CAM_H, self._CAM_W, 3), self._DARK, dtype=np.uint8)
            cv2.putText(img, "NO CAMERA FEED", (140, 160),
                        self._FONT, 0.8, self._GRAY, 1, cv2.LINE_AA)

        # Başlık çubuğu
        cv2.rectangle(img, (0, 0), (self._CAM_W, 28), (0, 0, 0), -1)
        cv2.putText(img, "CAMERA VIEW  (Lane Overlay)", (10, 20),
                    self._FONT, 0.5, self._ACCENT, 1, cv2.LINE_AA)

        # Durum LED
        det_col = self._GREEN if lane.detected else self._RED
        det_txt = "DETECTED" if lane.detected else "LOST"
        cv2.circle(img, (self._CAM_W - 18, 14), 6, det_col, -1)
        cv2.putText(img, det_txt, (self._CAM_W - 105, 20),
                    self._FONT, 0.4, det_col, 1, cv2.LINE_AA)

        canvas[0:self._CAM_H, 0:self._CAM_W] = img

    # =================================================================
    #  WARPED PANELİ
    # =================================================================

    def _draw_warped_panel(self, canvas, lane: LaneResult):
        """Sağ üst: Kuşbakışı sliding window görünümü."""
        if lane.warped_frame is not None:
            img = cv2.resize(lane.warped_frame, (self._WARP_W, self._WARP_H))
        else:
            img = np.full((self._WARP_H, self._WARP_W, 3), self._DARK, dtype=np.uint8)
            cv2.putText(img, "NO DATA", (130, 160),
                        self._FONT, 0.7, self._GRAY, 1, cv2.LINE_AA)

        # Başlık çubuğu
        cv2.rectangle(img, (0, 0), (self._WARP_W, 28), (0, 0, 0), -1)
        cv2.putText(img, "BIRD'S-EYE  (Sliding Window)", (10, 20),
                    self._FONT, 0.5, self._ACCENT, 1, cv2.LINE_AA)

        # Güven yüzdesi
        conf_pct = lane.confidence * 100
        conf_col = self._GREEN if conf_pct > 60 else self._YELLOW if conf_pct > 30 else self._RED
        cv2.putText(img, f"{conf_pct:.0f}%", (self._WARP_W - 55, 20),
                    self._FONT, 0.5, conf_col, 1, cv2.LINE_AA)

        canvas[0:self._WARP_H, self._CAM_W:self._TOTAL_W] = img

    # =================================================================
    #  TELEMETRİ PANELİ
    # =================================================================

    def _draw_telemetry(self, canvas, lane: LaneResult, speed_kmh: float,
                        steer: float, frame_no: int, lane_state: str,
                        target_offset: float, tl_state: str = "none",
                        tl_confirmed: bool = False):
        """Alt panel: hız, offset, steer, kontroller."""
        y0 = self._CAM_H
        panel = canvas[y0:y0 + self._DASH_H, 0:self._TOTAL_W]
        panel[:] = self._PANEL

        # Ayırıcı çizgi
        cv2.line(canvas, (0, y0), (self._TOTAL_W, y0), self._ACCENT, 2)

        # ── Satır 1: Hız + Offset + Eğrilik + Güven ─────────────────
        r1_y = y0 + 30
        cv2.putText(canvas, "SPEED", (15, r1_y), self._FONT, 0.4, self._GRAY, 1, cv2.LINE_AA)
        cv2.putText(canvas, f"{speed_kmh:.1f} km/h", (80, r1_y), self._FONT, 0.55, self._WHITE, 1, cv2.LINE_AA)

        cv2.putText(canvas, "OFFSET", (220, r1_y), self._FONT, 0.4, self._GRAY, 1, cv2.LINE_AA)
        off_col = self._GREEN if abs(lane.lateral_offset_m) < 0.3 else self._YELLOW if abs(lane.lateral_offset_m) < 0.8 else self._RED
        cv2.putText(canvas, f"{lane.lateral_offset_m:+.2f} m", (290, r1_y), self._FONT, 0.55, off_col, 1, cv2.LINE_AA)

        cv2.putText(canvas, "CURV", (430, r1_y), self._FONT, 0.4, self._GRAY, 1, cv2.LINE_AA)
        curv_txt = f"{lane.curvature_m:.0f} m" if lane.curvature_m < 9999 else "∞"
        cv2.putText(canvas, curv_txt, (490, r1_y), self._FONT, 0.55, self._CYAN, 1, cv2.LINE_AA)

        cv2.putText(canvas, "CONF", (620, r1_y), self._FONT, 0.4, self._GRAY, 1, cv2.LINE_AA)
        conf_pct = lane.confidence * 100
        conf_col = self._GREEN if conf_pct > 60 else self._YELLOW if conf_pct > 30 else self._RED
        cv2.putText(canvas, f"{conf_pct:.0f}%", (680, r1_y), self._FONT, 0.55, conf_col, 1, cv2.LINE_AA)

        # Frame sayacı
        cv2.putText(canvas, f"F:{frame_no}", (850, r1_y), self._FONT, 0.4, self._GRAY, 1, cv2.LINE_AA)

        # ── Satır 2: Direksiyon çubuğu + Şerit durumu ───────────────
        r2_y = y0 + 65

        # Direksiyon çubuğu
        bar_cx = 200
        bar_max = 150
        cv2.putText(canvas, "STEER", (15, r2_y), self._FONT, 0.4, self._GRAY, 1, cv2.LINE_AA)

        # Arka plan çubuk
        cv2.line(canvas, (bar_cx - bar_max, r2_y), (bar_cx + bar_max, r2_y), self._GRAY, 2)
        cv2.circle(canvas, (bar_cx, r2_y), 3, self._WHITE, -1)

        # Direksiyon pozisyonu
        bar_x = bar_cx + int(steer * bar_max)
        bar_col = self._GREEN if abs(steer) < 0.1 else self._YELLOW if abs(steer) < 0.25 else self._RED
        cv2.line(canvas, (bar_cx, r2_y), (bar_x, r2_y), bar_col, 4)
        cv2.circle(canvas, (bar_x, r2_y), 6, bar_col, -1)

        cv2.putText(canvas, "L", (bar_cx - bar_max - 14, r2_y + 5),
                    self._FONT, 0.35, self._GRAY, 1, cv2.LINE_AA)
        cv2.putText(canvas, "R", (bar_cx + bar_max + 5, r2_y + 5),
                    self._FONT, 0.35, self._GRAY, 1, cv2.LINE_AA)

        # Steer değeri
        cv2.putText(canvas, f"{steer:+.3f}", (380, r2_y + 5),
                    self._FONT, 0.5, bar_col, 1, cv2.LINE_AA)

        # Şerit değiştirme durumu
        cv2.putText(canvas, "LANE", (510, r2_y), self._FONT, 0.4, self._GRAY, 1, cv2.LINE_AA)
        state_col = self._CYAN if lane_state == "CENTER" else self._ORANGE
        cv2.putText(canvas, lane_state, (570, r2_y), self._FONT, 0.55, state_col, 1, cv2.LINE_AA)

        # Target offset
        if abs(target_offset) > 0.01:
            cv2.putText(canvas, f"TGT:{target_offset:+.1f}m", (700, r2_y),
                        self._FONT, 0.4, self._ORANGE, 1, cv2.LINE_AA)

        # Trafik ışığı durumu
        if tl_state != "none":
            tl_txt = "TL: RED" if tl_state == "red" else "TL: GREEN"
            if tl_confirmed: tl_txt += " (!)"
            tl_col = self._RED if tl_state == "red" else self._GREEN
            cv2.putText(canvas, tl_txt, (810, r2_y), self._FONT, 0.4, tl_col, 1, cv2.LINE_AA)

        # ── Satır 3: Kontrol bilgisi ─────────────────────────────────
        r3_y = y0 + 100
        cv2.putText(canvas, "A Left   D Right   |   1 Sunny   2 Rain   3 Snow   |   R Red   G Green   |   ESC Exit",
                    (15, r3_y), self._FONT_S, 0.9, self._GRAY, 1, cv2.LINE_AA)

        # ── Güven barı ───────────────────────────────────────────────
        bar_w = 200
        bar_h = 8
        bar_x0 = 740
        bar_y0 = y0 + 95
        # Arka plan
        cv2.rectangle(canvas, (bar_x0, bar_y0), (bar_x0 + bar_w, bar_y0 + bar_h), self._GRAY, 1)
        # Doluluk
        fill_w = int(bar_w * min(lane.confidence, 1.0))
        if fill_w > 0:
            cv2.rectangle(canvas, (bar_x0, bar_y0), (bar_x0 + fill_w, bar_y0 + bar_h), conf_col, -1)
        cv2.putText(canvas, "CONFIDENCE", (bar_x0, bar_y0 - 4),
                    self._FONT_S, 0.8, self._GRAY, 1, cv2.LINE_AA)

        # ── Alt kenarlık ─────────────────────────────────────────────
        r4_y = y0 + self._DASH_H - 15
        cv2.putText(canvas, "AUTONEX Lane Following System",
                    (self._TOTAL_W // 2 - 150, r4_y),
                    self._FONT_S, 0.9, (60, 60, 70), 1, cv2.LINE_AA)
