"""
views/traffic_light_panel.py — Trafik Işığı Sunum Paneli (Kompakt)

Düzen:
  ┌───────────────────────────┬────────────────────┐
  │  KAMERA + HSV MASKELERI   │  DURUM GÖSTERGESİ  │
  │  (Overlay + ROI + Kontur) │  ● Işık sembolü    │
  │                           │  Metrikler          │
  │  ┌─────────┬─────────┐   │  Araç kararı        │
  │  │ RED HSV │ GRN HSV │   │                      │
  │  └─────────┴─────────┘   │                      │
  └───────────────────────────┴────────────────────┘

Kullanım:
    panel = TrafficLightPanel()
    panel.render(tl_result, raw_frame, speed_kmh, ctrl)
    ...
    panel.destroy()
"""

import cv2
import numpy as np
from typing import Optional


class TrafficLightPanel:
    """
    Trafik ışığı tespitini kompakt görselleştiren OpenCV penceresi.
    """

    # ── Panel boyutları ──────────────────────────────────────────────
    _LEFT_W, _LEFT_H = 480, 300     # Sol: kamera + küçük maskeler
    _RIGHT_W, _RIGHT_H = 220, 300   # Sağ: durum göstergesi

    _TOTAL_W = _LEFT_W + _RIGHT_W   # 700
    _TOTAL_H = 300

    _CAM_H = 160                     # Ana kamera yüksekliği
    _MASK_H = 140                    # Alt maske paneli yüksekliği

    # ── Renkler (BGR) ────────────────────────────────────────────────
    _BG         = (18, 18, 28)
    _PANEL      = (30, 30, 45)
    _RED_C      = (60, 60, 220)
    _RED_DIM    = (30, 30, 100)
    _GREEN_C    = (50, 200, 80)
    _GREEN_DIM  = (20, 80, 30)
    _WHITE      = (220, 220, 220)
    _GRAY       = (110, 110, 125)
    _DARK_GRAY  = (55, 55, 70)
    _ACCENT     = (255, 180, 50)
    _CYAN       = (255, 210, 0)
    _FONT       = cv2.FONT_HERSHEY_SIMPLEX
    _FONT_S     = cv2.FONT_HERSHEY_PLAIN

    _WIN = "Traffic Light Detector"

    def __init__(self, window_x: int = 0, window_y: int = 600):
        cv2.namedWindow(self._WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._WIN, self._TOTAL_W, self._TOTAL_H)
        cv2.moveWindow(self._WIN, window_x, window_y)
        self._frame_count = 0
        self._anim_phase = 0

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def render(self, tl_result, raw_frame, speed_kmh: float = 0.0,
               ctrl=None) -> bool:
        self._frame_count += 1
        self._anim_phase = (self._anim_phase + 1) % 30

        canvas = np.full((self._TOTAL_H, self._TOTAL_W, 3),
                         self._BG, dtype=np.uint8)

        # ── Sol: Kamera + Maskeler ───────────────────────────────────
        self._draw_camera(canvas, tl_result, raw_frame)
        self._draw_masks(canvas, tl_result)

        # ── Sağ: Durum göstergesi ────────────────────────────────────
        self._draw_status(canvas, tl_result, speed_kmh, ctrl)

        cv2.imshow(self._WIN, canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            return False

        try:
            if cv2.getWindowProperty(self._WIN, cv2.WND_PROP_VISIBLE) < 1:
                return False
        except cv2.error:
            return False

        return True

    def destroy(self):
        try:
            cv2.destroyWindow(self._WIN)
        except Exception:
            pass

    # =================================================================
    #  SOL ÜST: KAMERA (overlay + ROI + konturlar)
    # =================================================================

    def _draw_camera(self, canvas, tl_result, raw_frame):
        x0, y0 = 0, 0
        w, h = self._LEFT_W, self._CAM_H

        if raw_frame is not None:
            img = cv2.resize(raw_frame, (w, h))

            # ROI sınırı — ince kesik çizgi
            if tl_result.roi_h > 0:
                orig_h = raw_frame.shape[0]
                roi_scaled = int(tl_result.roi_h * h / orig_h)
                for xx in range(0, w, 16):
                    cv2.line(img, (xx, roi_scaled), (min(xx + 8, w), roi_scaled),
                             (0, 100, 255), 1)

            # Kırmızı konturlar
            if tl_result.red_contours:
                orig_h, orig_w = raw_frame.shape[:2]
                sx, sy = w / orig_w, h / orig_h
                for cnt in tl_result.red_contours:
                    scaled = (cnt * [sx, sy]).astype(np.int32)
                    cv2.drawContours(img, [scaled], -1, (50, 50, 255), 2)
                    M = cv2.moments(scaled)
                    if M["m00"] > 1:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(img, (cx, cy), 4, (50, 50, 255), -1)

            # Yeşil konturlar
            if tl_result.green_contours:
                orig_h, orig_w = raw_frame.shape[:2]
                sx, sy = w / orig_w, h / orig_h
                for cnt in tl_result.green_contours:
                    scaled = (cnt * [sx, sy]).astype(np.int32)
                    cv2.drawContours(img, [scaled], -1, (50, 220, 80), 2)
                    M = cv2.moments(scaled)
                    if M["m00"] > 1:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(img, (cx, cy), 4, (50, 220, 80), -1)
        else:
            img = np.full((h, w, 3), self._PANEL, dtype=np.uint8)
            cv2.putText(img, "NO CAMERA", (w // 2 - 60, h // 2),
                        self._FONT, 0.6, self._GRAY, 1, cv2.LINE_AA)

        # Başlık çubuğu (ince)
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 22), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)
        cv2.putText(img, "CAMERA  |  ROI + CONTOURS", (8, 15),
                    self._FONT, 0.38, self._ACCENT, 1, cv2.LINE_AA)

        canvas[y0:y0 + h, x0:x0 + w] = img
        cv2.rectangle(canvas, (x0, y0), (x0 + w - 1, y0 + h - 1), self._DARK_GRAY, 1)

    # =================================================================
    #  SOL ALT: HSV MASKELER (yan yana kırmızı + yeşil)
    # =================================================================

    def _draw_masks(self, canvas, tl_result):
        x0, y0 = 0, self._CAM_H
        w, h = self._LEFT_W, self._MASK_H
        half_w = w // 2

        img = np.full((h, w, 3), self._PANEL, dtype=np.uint8)

        # Kırmızı maske (sol)
        if tl_result.red_mask is not None:
            rm = cv2.resize(tl_result.red_mask, (half_w, h - 20))
            rm_colored = np.zeros((h - 20, half_w, 3), dtype=np.uint8)
            rm_colored[:, :, 2] = rm
            img[20:h, 0:half_w] = rm_colored
        else:
            cv2.putText(img, "--", (half_w // 2 - 10, h // 2 + 10),
                        self._FONT, 0.5, self._GRAY, 1, cv2.LINE_AA)

        # Ayırıcı
        cv2.line(img, (half_w, 0), (half_w, h), self._DARK_GRAY, 1)

        # Yeşil maske (sağ)
        if tl_result.green_mask is not None:
            gm = cv2.resize(tl_result.green_mask, (half_w, h - 20))
            gm_colored = np.zeros((h - 20, half_w, 3), dtype=np.uint8)
            gm_colored[:, :, 1] = gm
            img[20:h, half_w:w] = gm_colored
        else:
            cv2.putText(img, "--", (half_w + half_w // 2 - 10, h // 2 + 10),
                        self._FONT, 0.5, self._GRAY, 1, cv2.LINE_AA)

        # Başlıklar (ince)
        cv2.rectangle(img, (0, 0), (w, 18), (0, 0, 0), -1)
        cv2.putText(img, "RED MASK", (8, 13),
                    self._FONT, 0.34, (80, 80, 255), 1, cv2.LINE_AA)
        cv2.putText(img, "GREEN MASK", (half_w + 6, 13),
                    self._FONT, 0.34, (50, 200, 80), 1, cv2.LINE_AA)

        canvas[y0:y0 + h, x0:x0 + w] = img
        cv2.rectangle(canvas, (x0, y0), (x0 + w - 1, y0 + h - 1), self._DARK_GRAY, 1)

    # =================================================================
    #  SAĞ: DURUM GÖSTERGESİ
    # =================================================================

    def _draw_status(self, canvas, tl_result, speed_kmh: float, ctrl):
        x0, y0 = self._LEFT_W, 0
        w, h = self._RIGHT_W, self._TOTAL_H
        cx = w // 2

        img = np.full((h, w, 3), self._PANEL, dtype=np.uint8)

        state = tl_result.state
        confirmed = tl_result.confirmed
        blink = self._anim_phase < 15

        # ── Trafik ışığı sembolü (3 daire — kompakt) ─────────────────
        light_x = cx
        light_top = 10
        r = 16
        spacing = 40
        box_w = 60
        box_h = spacing * 3 + 10

        # Arka plan kutu
        cv2.rectangle(img, (cx - box_w // 2, light_top),
                      (cx + box_w // 2, light_top + box_h),
                      (20, 20, 35), -1)
        cv2.rectangle(img, (cx - box_w // 2, light_top),
                      (cx + box_w // 2, light_top + box_h),
                      self._DARK_GRAY, 1)

        circles = [
            (light_top + 25,  "red"),
            (light_top + 65,  "yellow"),
            (light_top + 105, "green"),
        ]

        for (cy_dot, color_name) in circles:
            if color_name == "red":
                if state == "red":
                    col = self._RED_C if (not confirmed or blink) else self._RED_DIM
                    if confirmed:
                        cv2.circle(img, (light_x, cy_dot), r + 4, (30, 30, 100), 2)
                else:
                    col = self._RED_DIM
            elif color_name == "yellow":
                col = (20, 70, 100)
            else:
                if state == "green":
                    col = self._GREEN_C if blink else self._GREEN_DIM
                    cv2.circle(img, (light_x, cy_dot), r + 4, (20, 80, 30), 2)
                else:
                    col = self._GREEN_DIM
            cv2.circle(img, (light_x, cy_dot), r, col, -1)
            cv2.circle(img, (light_x, cy_dot), r, self._DARK_GRAY, 1)

        # ── Durum metni ──────────────────────────────────────────────
        status_y = light_top + box_h + 20
        if state == "red":
            s_txt = "KIRMIZI"
            s_col = (80, 80, 255)
            sub_txt = "ONAYLANDI" if confirmed else "Dogrulaniyor..."
            sub_col = (80, 80, 255) if confirmed else self._GRAY
        elif state == "green":
            s_txt = "YESIL"
            s_col = self._GREEN_C
            sub_txt = "Yol acik"
            sub_col = self._GREEN_C
        else:
            s_txt = "---"
            s_col = self._GRAY
            sub_txt = "Taranıyor"
            sub_col = self._GRAY

        txt_sz = cv2.getTextSize(s_txt, self._FONT, 0.55, 2)[0]
        cv2.putText(img, s_txt, (cx - txt_sz[0] // 2, status_y),
                    self._FONT, 0.55, s_col, 2, cv2.LINE_AA)
        sub_sz = cv2.getTextSize(sub_txt, self._FONT, 0.35, 1)[0]
        cv2.putText(img, sub_txt, (cx - sub_sz[0] // 2, status_y + 18),
                    self._FONT, 0.35, sub_col, 1, cv2.LINE_AA)

        # ── Metrikler (kompakt) ──────────────────────────────────────
        m_y = status_y + 36
        self._metric_line(img, "Alan", f"{tl_result.area} px", m_y, w)
        self._metric_line(img, "Hiz", f"{speed_kmh:.0f} km/h", m_y + 20, w)

        # ── Araç kararı ──────────────────────────────────────────────
        dec_y = h - 50
        cv2.line(img, (8, dec_y - 6), (w - 8, dec_y - 6), self._DARK_GRAY, 1)

        if tl_result.should_stop:
            dec_col = self._RED_C
            dec_txt = "FREN"
            brake_val = tl_result.brake_intensity()
            dec_sub = f"{brake_val:.0%}"
            box_col = self._RED_C if blink else self._RED_DIM
            cv2.rectangle(img, (6, dec_y - 2), (w - 6, h - 4), box_col, 2)
        elif state == "green":
            dec_col = self._GREEN_C
            dec_txt = "DEVAM"
            dec_sub = f"{speed_kmh:.0f} km/h"
            cv2.rectangle(img, (6, dec_y - 2), (w - 6, h - 4), self._GREEN_C, 1)
        else:
            dec_col = self._GRAY
            dec_txt = "IZLIYOR"
            dec_sub = ""
            cv2.rectangle(img, (6, dec_y - 2), (w - 6, h - 4), self._DARK_GRAY, 1)

        txt_sz2 = cv2.getTextSize(dec_txt, self._FONT, 0.5, 2)[0]
        cv2.putText(img, dec_txt, (cx - txt_sz2[0] // 2, dec_y + 20),
                    self._FONT, 0.5, dec_col, 2, cv2.LINE_AA)
        if dec_sub:
            sub_sz2 = cv2.getTextSize(dec_sub, self._FONT, 0.35, 1)[0]
            cv2.putText(img, dec_sub, (cx - sub_sz2[0] // 2, dec_y + 37),
                        self._FONT, 0.35, dec_col, 1, cv2.LINE_AA)

        canvas[y0:y0 + h, x0:x0 + w] = img
        cv2.rectangle(canvas, (x0, y0), (x0 + w - 1, y0 + h - 1), self._DARK_GRAY, 1)

    # =================================================================
    #  YARDIMCI
    # =================================================================

    def _metric_line(self, img, label: str, value: str, y: int, panel_w: int):
        cv2.putText(img, label, (10, y),
                    self._FONT, 0.33, self._GRAY, 1, cv2.LINE_AA)
        val_sz = cv2.getTextSize(value, self._FONT, 0.36, 1)[0]
        cv2.putText(img, value, (panel_w - val_sz[0] - 10, y),
                    self._FONT, 0.36, self._CYAN, 1, cv2.LINE_AA)
