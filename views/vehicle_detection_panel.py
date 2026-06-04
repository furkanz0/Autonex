"""
views/vehicle_detection_panel.py — OpenCV Araç Tespiti Debug Paneli

Trafik ışığı paneline (traffic_light_panel.py) benzer yapıda,
araç tespiti ve ACC durumunu gösteren ayrı bir OpenCV penceresi.

Layout (500×380):
  ┌─────────────────────────────────────────┐
  │ [Kamera görüntüsü + bounding box'lar]   │
  │                               340×220   │
  ├─────────────────────────────────────────┤
  │ ACC: [durum]  Hız: 28 km/h             │
  │ En yakın: 12.3m   Araç sayısı: 2       │
  │ [Mesafe çubuğu]                         │
  │ [Araç listesi]                          │
  └─────────────────────────────────────────┘
"""

import cv2
import numpy as np

from config import VD_SAFE_DIST_M, VD_CRITICAL_DIST_M, VD_EMERGENCY_DIST_M


class VehicleDetectionPanel:
    """
    Araç tespiti ve ACC durumu için OpenCV debug penceresi.

    Trafik ışığı panelinin yanına yerleştirilir.
    """

    # Panel boyutları
    PANEL_W = 500
    PANEL_H = 400
    CAM_W   = 480
    CAM_H   = 220

    # Renkler (BGR)
    C_BG        = (28, 30, 35)
    C_FREE      = (80, 200, 80)
    C_FOLLOWING = (30, 200, 230)
    C_BRAKING   = (30, 110, 230)
    C_EMERGENCY = (30, 30, 200)
    C_TEXT      = (200, 205, 215)
    C_DIM       = (100, 105, 115)
    C_TITLE     = (150, 190, 255)
    C_BAR_BG    = (50, 52, 58)

    def __init__(self, window_x: int = 560, window_y: int = 530):
        self._win = "Vehicle Detection"
        cv2.namedWindow(self._win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._win, self.PANEL_W, self.PANEL_H)
        try:
            cv2.moveWindow(self._win, window_x, window_y)
        except Exception:
            pass
        self._canvas = np.zeros((self.PANEL_H, self.PANEL_W, 3), dtype=np.uint8)

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def render(self, vehicle_result, camera_frame, speed_kmh: float,
               acc_decision=None) -> bool:
        """
        Paneli güncelle ve göster.

        Args:
            vehicle_result: VehicleDetectionResult
            camera_frame: BGR kamera görüntüsü (overlay ile)
            speed_kmh: Anlık hız
            acc_decision: ACCDecision (opsiyonel)

        Returns:
            False → pencere kapatıldı
        """
        canvas = np.full((self.PANEL_H, self.PANEL_W, 3), self.C_BG, dtype=np.uint8)

        # ── Kamera görüntüsü (overlay ile) ──────────────────────────
        if camera_frame is not None:
            frame = vehicle_result.overlay_frame if vehicle_result.overlay_frame is not None else camera_frame
            cam_resized = cv2.resize(frame, (self.CAM_W, self.CAM_H))
            y0 = 8
            canvas[y0:y0 + self.CAM_H, 10:10 + self.CAM_W] = cam_resized

        # ── ACC Durum etiketi ────────────────────────────────────────
        y_info = self.CAM_H + 16
        status = vehicle_result.status
        status_color = self._state_color(status)

        cv2.putText(canvas, "ACC", (12, y_info + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.C_DIM, 1, cv2.LINE_AA)
        cv2.putText(canvas, status, (60, y_info + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2, cv2.LINE_AA)

        # Hız
        cv2.putText(canvas, f"{speed_kmh:.1f} km/h", (240, y_info + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.C_TEXT, 1, cv2.LINE_AA)

        # ── Mesafe + araç sayısı ──────────────────────────────────────
        y_dist = y_info + 28
        if vehicle_result.detected:
            dist_str = f"{vehicle_result.closest_distance_m:.1f}m"
            dist_color = self._dist_color(vehicle_result.closest_distance_m)
            cv2.putText(canvas, "En yakın:", (12, y_dist),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, self.C_DIM, 1, cv2.LINE_AA)
            cv2.putText(canvas, dist_str, (100, y_dist),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, dist_color, 2, cv2.LINE_AA)
        else:
            cv2.putText(canvas, "Araç yok", (12, y_dist),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_FREE, 1, cv2.LINE_AA)

        cnt_str = f"Tespit: {vehicle_result.vehicle_count}"
        cv2.putText(canvas, cnt_str, (280, y_dist),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.C_DIM, 1, cv2.LINE_AA)

        # ── Mesafe çubuğu (progress bar) ─────────────────────────────
        y_bar = y_dist + 20
        self._draw_distance_bar(canvas, y_bar, vehicle_result.closest_distance_m)

        # ── Araç listesi ─────────────────────────────────────────────
        y_list = y_bar + 38
        self._draw_vehicle_list(canvas, y_list, vehicle_result)

        # ── ACC karar detayı ─────────────────────────────────────────
        if acc_decision is not None and acc_decision.active:
            y_acc = self.PANEL_H - 30
            cv2.putText(canvas,
                        f"thr={acc_decision.throttle:.2f}  br={acc_decision.brake:.2f}",
                        (12, y_acc),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, self.C_DIM, 1, cv2.LINE_AA)

        self._canvas = canvas
        cv2.imshow(self._win, canvas)
        key = cv2.waitKey(1) & 0xFF
        return key != ord("q") and cv2.getWindowProperty(self._win, cv2.WND_PROP_VISIBLE) >= 1

    def destroy(self):
        """Pencereyi kapat."""
        try:
            cv2.destroyWindow(self._win)
        except Exception:
            pass

    # =================================================================
    #  YARDIMCI ÇİZİM
    # =================================================================

    def _draw_distance_bar(self, canvas: np.ndarray, y: int, dist: float):
        """Güvenli mesafe göstergesi (yeşil → kırmızı çubuk)."""
        bar_x, bar_w, bar_h = 12, 460, 16

        # Arka plan
        cv2.rectangle(canvas, (bar_x, y), (bar_x + bar_w, y + bar_h),
                      self.C_BAR_BG, -1)
        cv2.rectangle(canvas, (bar_x, y), (bar_x + bar_w, y + bar_h),
                      (70, 72, 80), 1)

        # Dolgu oranı: 0=çok yakın (tam), 1=uzak (boş)
        max_display = VD_SAFE_DIST_M * 1.5
        ratio = max(0.0, min(1.0, 1.0 - (dist / max_display)))

        if ratio > 0.01:
            fill_w = int(bar_w * ratio)
            bar_color = self._dist_color(dist)
            cv2.rectangle(canvas, (bar_x, y), (bar_x + fill_w, y + bar_h),
                          bar_color, -1)

        # Eşik işaretleri
        for threshold, label in [
            (VD_EMERGENCY_DIST_M, "!"),
            (VD_CRITICAL_DIST_M, "⚠"),
            (VD_SAFE_DIST_M, "●"),
        ]:
            tx = bar_x + int(bar_w * (1.0 - threshold / max_display))
            cv2.line(canvas, (tx, y), (tx, y + bar_h), (150, 150, 160), 1)

        # Mesafe etiketi
        label_txt = f"0m   EMRG({VD_EMERGENCY_DIST_M:.0f})  CRIT({VD_CRITICAL_DIST_M:.0f})  SAFE({VD_SAFE_DIST_M:.0f})  {max_display:.0f}m"
        cv2.putText(canvas, label_txt, (bar_x, y + bar_h + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, self.C_DIM, 1, cv2.LINE_AA)

    def _draw_vehicle_list(self, canvas: np.ndarray, y_start: int,
                            vehicle_result):
        """Tespit edilen araçları liste olarak göster."""
        if not vehicle_result.vehicles:
            return

        cv2.putText(canvas, "Tespit edilen araçlar:", (12, y_start),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.C_DIM, 1, cv2.LINE_AA)

        for i, v in enumerate(vehicle_result.vehicles[:4]):
            y = y_start + 16 + i * 16
            color = self._dist_color(v.distance_m)
            txt = (f"  #{i+1}  {v.distance_m:.1f}m  "
                   f"bbox=({v.w}×{v.h})  conf={v.confidence:.2f}")
            cv2.putText(canvas, txt, (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv2.LINE_AA)

    # =================================================================
    #  RENK YARDIMCILARI
    # =================================================================

    def _state_color(self, status: str) -> tuple:
        return {
            "FREE":      self.C_FREE,
            "FOLLOWING": self.C_FOLLOWING,
            "BRAKING":   self.C_BRAKING,
            "EMERGENCY": self.C_EMERGENCY,
        }.get(status, self.C_TEXT)

    def _dist_color(self, dist: float) -> tuple:
        if dist < VD_EMERGENCY_DIST_M:
            return self.C_EMERGENCY
        if dist < VD_CRITICAL_DIST_M:
            return self.C_BRAKING
        if dist < VD_SAFE_DIST_M:
            return self.C_FOLLOWING
        return self.C_FREE
