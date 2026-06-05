"""
models/lane_detector.py — OpenCV tabanlı şerit tespit modülü

Pipeline:
  1. BGR → HLS renk uzayı dönüşümü
  2. Beyaz + Sarı şerit maskeleri
  3. Canny kenar tespiti
  4. Birleşik maske → ROI kırpma
  5. Perspective Warp → kuşbakışı (bird's-eye)
  6. Sliding Window → şerit piksel tespiti
  7. 2. derece polinom fit
  8. Piksel → metre dönüşümü (offset + eğrilik)
  9. Overlay frame oluşturma

Kullanım:
    detector = LaneDetector()
    result   = detector.process(bgr_frame)   # → LaneResult
"""

from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
import os

from config import (
    LANE_CAM_W, LANE_CAM_H,
    WHITE_L_THRESH, WHITE_S_THRESH,
    YELLOW_H_THRESH, YELLOW_S_THRESH, YELLOW_L_THRESH,
    CANNY_LOW, CANNY_HIGH,
    WARP_SRC, WARP_DST,
    SW_NWINDOWS, SW_MARGIN, SW_MINPIX,
    LANE_XM_PER_PIX, LANE_YM_PER_PIX,
)


# =====================================================================
#  DATA CLASS — Tespit Sonucu
# =====================================================================

@dataclass
class LaneResult:
    """Tek bir frame'in şerit tespit sonucu."""
    detected: bool = False
    lateral_offset_m: float = 0.0       # + = şerit merkezi sağda → araç solda
    curvature_m: float = float("inf")   # eğrilik yarıçapı (metre)
    confidence: float = 0.0             # 0.0 – 1.0
    left_fit: Optional[np.ndarray] = field(default=None, repr=False)
    right_fit: Optional[np.ndarray] = field(default=None, repr=False)
    overlay_frame: Optional[np.ndarray] = field(default=None, repr=False)
    warped_frame: Optional[np.ndarray] = field(default=None, repr=False)


# =====================================================================
#  ANA SINIF
# =====================================================================

class LaneDetector:
    """
    Kamera frame'inden şerit tespit eden OpenCV pipeline.

    Her process() çağrısında tam pipeline çalışır ve LaneResult döndürür.
    """

    # Minimum piksel sayısı — bunun altında "şerit yok" sayılır
    _MIN_LANE_PIXELS = 600

    def __init__(self):
        # ── Perspektif dönüşüm matrisleri ────────────────────────────
        src = np.float32(WARP_SRC)
        dst = np.float32(WARP_DST)
        self._M = cv2.getPerspectiveTransform(src, dst)
        self._M_inv = cv2.getPerspectiveTransform(dst, src)

        # ── Önceki frame katsayıları (smoothing) ─────────────────────
        self._left_fit = None
        self._right_fit = None

        # ── Sayaç ────────────────────────────────────────────────────
        self._frame_count = 0

        # ── Debug dizini ─────────────────────────────────────────────
        self._debug_enabled = os.getenv("AUTONEX_LANE_DEBUG") == "1"
        self._debug_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "lane_debug")
        if self._debug_enabled:
            os.makedirs(self._debug_dir, exist_ok=True)

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def reset(self):
        """Clear cached lane fits so the next frame locks onto the current lane."""
        self._left_fit = None
        self._right_fit = None

    def process(self, frame, draw_debug: bool = True) -> LaneResult:
        """
        Tek bir BGR frame'i işleyip LaneResult döndürür.

        Args:
            frame: np.ndarray BGR (H, W, 3) veya None

        Returns:
            LaneResult
        """
        if frame is None:
            return LaneResult()

        self._frame_count += 1
        h, w = frame.shape[:2]

        # ── 1. Renk filtresi ─────────────────────────────────────────
        binary = self._color_threshold(frame)

        # ── 2. Canny kenar tespiti ───────────────────────────────────
        edges = self._canny_edges(frame)

        # ── 3. Birleşik maske ────────────────────────────────────────
        combined = cv2.bitwise_or(binary, edges)

        # ── 4. ROI ───────────────────────────────────────────────────
        roi = self._apply_roi(combined)

        # ── 5. Perspective Warp ──────────────────────────────────────
        warped = cv2.warpPerspective(roi, self._M, (w, h))

        # ── 6. Sliding Window ────────────────────────────────────────
        left_fit, right_fit, left_pts, right_pts, sw_img = \
            self._sliding_window(warped)

        # ── 7. Geçerlilik ve EMA Filtresi (Smoothing) ────────────────
        left_ok = left_pts is not None and len(left_pts[0]) > self._MIN_LANE_PIXELS
        right_ok = right_pts is not None and len(right_pts[0]) > self._MIN_LANE_PIXELS

        detected = False

        if left_ok and right_ok:
            self._left_fit = left_fit
            self._right_fit = right_fit
            detected = True
        elif left_ok and self._right_fit is not None:
            self._left_fit = left_fit
            detected = True
        elif right_ok and self._left_fit is not None:
            self._right_fit = right_fit
            detected = True
        elif self._left_fit is not None and self._right_fit is not None:
            detected = True  # önceki fit ile devam

        # ── 8. Offset + Eğrilik (Lookahead) ──────────────────────────
        offset_m = 0.0
        curvature_m = float("inf")
        confidence = 0.0

        if detected and self._left_fit is not None and self._right_fit is not None:
            offset_m = self._compute_offset_m(self._left_fit, self._right_fit, h, w)
            curvature_m = self._compute_curvature_m(self._left_fit, self._right_fit, h)
            confidence = self._compute_confidence(left_pts, right_pts, left_ok, right_ok)

        # ── 9. Overlay frame ────────────────────────────────────────
        if draw_debug:
            overlay = self._draw_overlay(frame, self._left_fit, self._right_fit, detected)
            warped_frame = sw_img
        else:
            overlay = None
            warped_frame = None

        # ── 10. Debug kayıt (her 50 frame) ───────────────────────────
        if self._debug_enabled and draw_debug and self._frame_count % 50 == 0:
            self._save_debug(overlay, sw_img)

        return LaneResult(
            detected=detected,
            lateral_offset_m=offset_m,
            curvature_m=curvature_m,
            confidence=confidence,
            left_fit=self._left_fit,
            right_fit=self._right_fit,
            overlay_frame=overlay,
            warped_frame=warped_frame,
        )

    # =================================================================
    #  RENK FİLTRESİ (HLS)
    # =================================================================

    def _color_threshold(self, frame):
        """BGR → HLS, beyaz ve sarı şerit maskeleri."""
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        h_ch, l_ch, s_ch = hls[:, :, 0], hls[:, :, 1], hls[:, :, 2]

        # Beyaz maske
        white = np.zeros_like(l_ch, dtype=np.uint8)
        white[
            (l_ch >= WHITE_L_THRESH[0]) & (l_ch <= WHITE_L_THRESH[1]) &
            (s_ch >= WHITE_S_THRESH[0]) & (s_ch <= WHITE_S_THRESH[1])
        ] = 255

        # Sarı maske
        yellow = np.zeros_like(l_ch, dtype=np.uint8)
        yellow[
            (h_ch >= YELLOW_H_THRESH[0]) & (h_ch <= YELLOW_H_THRESH[1]) &
            (s_ch >= YELLOW_S_THRESH[0]) & (s_ch <= YELLOW_S_THRESH[1]) &
            (l_ch >= YELLOW_L_THRESH[0]) & (l_ch <= YELLOW_L_THRESH[1])
        ] = 255

        return cv2.bitwise_or(white, yellow)

    # =================================================================
    #  CANNY KENAR TESPİTİ
    # =================================================================

    def _canny_edges(self, frame):
        """Grayscale → GaussianBlur → Canny."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)

    # =================================================================
    #  ROI (İLGİ BÖLGESİ)
    # =================================================================

    def _apply_roi(self, binary):
        """Alt yarı trapez ROI maskesi."""
        h, w = binary.shape[:2]
        mask = np.zeros_like(binary)
        polygon = np.array([[
            (0, h),
            (0, int(h * 0.5)),
            (w, int(h * 0.5)),
            (w, h),
        ]], dtype=np.int32)
        cv2.fillPoly(mask, polygon, 255)
        return cv2.bitwise_and(binary, mask)

    # =================================================================
    #  SLIDING WINDOW
    # =================================================================

    def _sliding_window(self, warped):
        """
        Histogram tabanlı kayan pencere şerit tespiti.

        Returns:
            left_fit, right_fit, left_pts, right_pts, debug_img
        """
        h, w = warped.shape[:2]
        histogram = np.sum(warped[h // 2:, :], axis=0)
        midpoint = w // 2

        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        out_img = np.dstack((warped, warped, warped))
        win_h = h // SW_NWINDOWS

        nonzero = warped.nonzero()
        nz_y = np.array(nonzero[0])
        nz_x = np.array(nonzero[1])

        left_current = left_base
        right_current = right_base
        left_lane_inds = []
        right_lane_inds = []

        for win_i in range(SW_NWINDOWS):
            y_low = h - (win_i + 1) * win_h
            y_high = h - win_i * win_h

            xl_low = left_current - SW_MARGIN
            xl_high = left_current + SW_MARGIN
            xr_low = right_current - SW_MARGIN
            xr_high = right_current + SW_MARGIN

            # Debug pencereler
            cv2.rectangle(out_img, (xl_low, y_low), (xl_high, y_high),
                          (100, 180, 140), 1)
            cv2.rectangle(out_img, (xr_low, y_low), (xr_high, y_high),
                          (100, 180, 140), 1)

            good_left = (
                (nz_y >= y_low) & (nz_y < y_high) &
                (nz_x >= xl_low) & (nz_x < xl_high)
            ).nonzero()[0]

            good_right = (
                (nz_y >= y_low) & (nz_y < y_high) &
                (nz_x >= xr_low) & (nz_x < xr_high)
            ).nonzero()[0]

            # --- Kavşak / Karelaj Filtresi ---
            # Bir pencere (120x40 = 4800 piksel) içinde çok fazla şerit pikseli varsa
            # bu muhtemelen yatay bir dur çizgisi veya kavşaktaki sarı taralı alandır.
            if len(good_left) > 1500:
                good_left = np.array([], dtype=np.int64)
            if len(good_right) > 1500:
                good_right = np.array([], dtype=np.int64)

            left_lane_inds.append(good_left)
            right_lane_inds.append(good_right)

            if len(good_left) > SW_MINPIX:
                left_current = int(np.mean(nz_x[good_left]))
            if len(good_right) > SW_MINPIX:
                right_current = int(np.mean(nz_x[good_right]))

        left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else np.array([])
        right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else np.array([])

        left_pts = None
        left_fit = None
        if len(left_lane_inds) > 0:
            lx = nz_x[left_lane_inds]
            ly = nz_y[left_lane_inds]
            left_pts = (ly, lx)
            if len(lx) > 50:
                try:
                    left_fit = np.polyfit(ly, lx, 2)
                except (np.linalg.LinAlgError, ValueError):
                    left_fit = None
            out_img[ly, lx] = [180, 100, 80]

        right_pts = None
        right_fit = None
        if len(right_lane_inds) > 0:
            rx = nz_x[right_lane_inds]
            ry = nz_y[right_lane_inds]
            right_pts = (ry, rx)
            if len(rx) > 50:
                try:
                    right_fit = np.polyfit(ry, rx, 2)
                except (np.linalg.LinAlgError, ValueError):
                    right_fit = None
            out_img[ry, rx] = [80, 80, 180]

        # Polinom eğrilerini çiz
        for fit, color in [(left_fit, (160, 190, 100)), (right_fit, (160, 190, 100))]:
            if fit is not None:
                plot_y = np.linspace(0, h - 1, h)
                plot_x = np.clip(
                    (fit[0] * plot_y**2 + fit[1] * plot_y + fit[2]).astype(int),
                    0, w - 1)
                pts = np.column_stack((plot_x, plot_y.astype(int)))
                cv2.polylines(out_img, [pts], False, color, 2)

        return left_fit, right_fit, left_pts, right_pts, out_img

    # =================================================================
    #  OFFSET HESAPLAMA (piksel → metre)
    # =================================================================

    def _compute_offset_m(self, left_fit, right_fit, h, w):
        """
        Şerit merkezinden sapma (metre).
        Pozitif = şerit merkezi sağda → araç solda → sağa dönmeli.
        Lookahead (İleriye bakış) mesafesindeki sapmayı hesaplar.
        """
        from config import LANE_LOOKAHEAD_M
        
        # Lookahead noktasını piksel cinsinden bul (y=0 resmin üstü, y=h-1 altı)
        lookahead_px = LANE_LOOKAHEAD_M / LANE_YM_PER_PIX
        y_eval = (h - 1) - lookahead_px
        y_eval = max(0, min(h - 1, y_eval))

        left_x = left_fit[0] * y_eval**2 + left_fit[1] * y_eval + left_fit[2]
        right_x = right_fit[0] * y_eval**2 + right_fit[1] * y_eval + right_fit[2]

        lane_center_px = (left_x + right_x) / 2.0
        image_center_px = w / 2.0

        return (lane_center_px - image_center_px) * LANE_XM_PER_PIX

    # =================================================================
    #  EĞRİLİK HESAPLAMA (metre)
    # =================================================================

    def _compute_curvature_m(self, left_fit, right_fit, h):
        """
        Eğrilik yarıçapı (metre).
        R = (1 + (2ay + b)²)^(3/2) / |2a|
        Piksel katsayılarını metre uzayına dönüştürüp hesaplar.
        """
        y_eval = h - 1

        # Piksel → metre katsayı dönüşümü
        # x_m = a_m * y_m² + b_m * y_m + c_m
        # a_m = a_px * xm/ym², b_m = b_px * xm/ym
        curvatures = []
        for fit in [left_fit, right_fit]:
            if fit is not None:
                a_m = fit[0] * LANE_XM_PER_PIX / (LANE_YM_PER_PIX ** 2)
                b_m = fit[1] * LANE_XM_PER_PIX / LANE_YM_PER_PIX
                y_m = y_eval * LANE_YM_PER_PIX

                denom = abs(2.0 * a_m)
                if denom > 1e-6:
                    R = ((1.0 + (2.0 * a_m * y_m + b_m) ** 2) ** 1.5) / denom
                    curvatures.append(min(R, 10000.0))  # cap at 10km

        if curvatures:
            return sum(curvatures) / len(curvatures)
        return float("inf")

    # =================================================================
    #  GÜVEN SKORU
    # =================================================================

    @staticmethod
    def _compute_confidence(left_pts, right_pts, left_ok, right_ok):
        """Tespit güveni (0.0 – 1.0): piksel sayısına göre."""
        score = 0.0
        max_pixels = 5000.0  # tam güven için beklenen piksel

        if left_ok and left_pts is not None:
            score += min(len(left_pts[0]) / max_pixels, 0.5)
        if right_ok and right_pts is not None:
            score += min(len(right_pts[0]) / max_pixels, 0.5)

        return min(score, 1.0)

    # =================================================================
    #  OVERLAY ÇIZIMI
    # =================================================================

    def _draw_overlay(self, frame, left_fit, right_fit, detected):
        """Kamera frame'i üzerine şerit alanını ve bilgi metinlerini çiz."""
        overlay = frame.copy()
        h, w = frame.shape[:2]

        if detected and left_fit is not None and right_fit is not None:
            # Yeşil şerit alanını kuşbakışı → orijinal perspektife çevir
            lane_fill = np.zeros_like(frame)
            plot_y = np.linspace(0, h - 1, h)
            left_x = np.clip(
                (left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]).astype(int),
                0, w - 1)
            right_x = np.clip(
                (right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]).astype(int),
                0, w - 1)

            pts_left = np.column_stack((left_x, plot_y.astype(int)))
            pts_right = np.flipud(np.column_stack((right_x, plot_y.astype(int))))
            fill_pts = np.vstack((pts_left, pts_right))
            cv2.fillPoly(lane_fill, [fill_pts], (60, 140, 70))

            # Ters warp
            unwarped = cv2.warpPerspective(lane_fill, self._M_inv, (w, h))
            overlay = cv2.addWeighted(overlay, 1.0, unwarped, 0.3, 0)

            # Şerit çizgilerini çiz
            for fit, color in [(left_fit, (80, 160, 200)), (right_fit, (200, 170, 80))]:
                line_img = np.zeros_like(frame)
                px = np.clip(
                    (fit[0] * plot_y**2 + fit[1] * plot_y + fit[2]).astype(int),
                    0, w - 1)
                pts = np.column_stack((px, plot_y.astype(int)))
                cv2.polylines(line_img, [pts], False, color, 2)
                unwarped_line = cv2.warpPerspective(line_img, self._M_inv, (w, h))
                overlay = cv2.addWeighted(overlay, 1.0, unwarped_line, 0.6, 0)

        # Warp kaynağı trapezini çiz
        src_pts = np.array(WARP_SRC, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(overlay, [src_pts], True, (120, 120, 90), 1)

        # Durum göstergesi
        status_txt = "LANE OK" if detected else "LANE LOST"
        status_col = (80, 190, 120) if detected else (100, 100, 200)
        cv2.circle(overlay, (w - 20, 20), 6, status_col, -1)
        cv2.putText(overlay, status_txt, (w - 130, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_col, 1, cv2.LINE_AA)

        return overlay

    # =================================================================
    #  DEBUG KAYIT
    # =================================================================

    def _save_debug(self, overlay, warped):
        """Overlay ve warped görüntüleri debug dizinine kaydet."""
        try:
            if overlay is not None:
                cv2.imwrite(os.path.join(self._debug_dir, "latest_overlay.jpg"), overlay)
            if warped is not None:
                cv2.imwrite(os.path.join(self._debug_dir, "latest_warped.jpg"), warped)
        except Exception:
            pass
