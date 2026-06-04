"""
controllers/traffic_light_controller.py — Kamera Tabanlı Trafik Işığı Tespiti

Tamamen OpenCV görüntü işleme ile çalışır. CARLA aktör API'si kullanılmaz.

Pipeline:
  1. ROI: Görüntünün üst %55'i (trafik ışıkları gökyüzüne yakın)
  2. BGR → HSV dönüşümü
  3. Kırmızı maske (iki HSV aralığı — kırmızı 0°/180° etrafında sarmalanır)
  4. Yeşil maske
  5. Morfolojik temizlik (gürültü azaltma)
  6. Kontur tespiti → alan + dairesellik filtresi
  7. Çoklu-frame doğrulama (false positive azaltma)

Kullanım:
    detector = CameraTrafficLightDetector()
    result   = detector.process(bgr_frame)    # → TrafficLightResult
    if result.should_stop:
        ctrl = result.brake_control(speed_kmh)
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

from config import (
    TL_RED_HSV_LOW1, TL_RED_HSV_HIGH1,
    TL_RED_HSV_LOW2, TL_RED_HSV_HIGH2,
    TL_GREEN_HSV_LOW, TL_GREEN_HSV_HIGH,
    TL_MIN_AREA, TL_MAX_AREA, TL_MIN_CIRCULARITY,
    TL_MIN_BBOX_PX, TL_MAX_BBOX_RATIO,
    TL_MIN_ASPECT_RATIO, TL_MAX_ASPECT_RATIO,
    TL_MIN_EXTENT, TL_MAX_EXTENT,
    TL_MIN_DARK_CONTEXT, TL_CONTEXT_PAD_RATIO,
    TL_CENTER_MARGIN_RATIO,
    TL_ROI_RATIO, TL_CONFIRM_FRAMES, TL_BRAKE_AREA_SCALE,
)


# =====================================================================
#  SONUÇ YAPISI
# =====================================================================

@dataclass
class TrafficLightResult:
    """Tek bir frame'in trafik ışığı tespit sonucu."""
    state: str = "none"             # "red", "green", "none"
    confirmed: bool = False         # çoklu-frame doğrulandı mı
    area: int = 0                   # en büyük kontur alanı (mesafe tahmini)
    center: Optional[Tuple[int, int]] = None  # (x, y) kontur merkezi
    contour_count: int = 0          # tespit edilen kontur sayısı
    overlay_frame: Optional[np.ndarray] = field(default=None, repr=False)
    # ── Sunum paneli için ek alanlar ──────────────────────────────────
    red_mask: Optional[np.ndarray] = field(default=None, repr=False)    # ham kırmızı HSV maskesi
    green_mask: Optional[np.ndarray] = field(default=None, repr=False)  # ham yeşil HSV maskesi
    roi_h: int = 0                  # ROI sınırı yüksekliği (piksel)
    red_contours: list = field(default_factory=list, repr=False)        # kırmızı kontur listesi
    green_contours: list = field(default_factory=list, repr=False)      # yeşil kontur listesi

    @property
    def should_stop(self) -> bool:
        """Araç durmalı mı?"""
        return self.state == "red" and self.confirmed

    def brake_intensity(self) -> float:
        """Fren yoğunluğu (0.0–0.9). Büyük alan = yakın ışık = sert fren."""
        if not self.should_stop:
            return 0.0
        return min(0.3 + (self.area / TL_BRAKE_AREA_SCALE), 0.9)


# =====================================================================
#  ANA SINIF
# =====================================================================

class CameraTrafficLightDetector:
    """
    Kamera görüntüsünden trafik ışığı tespit eden OpenCV modülü.

    CARLA aktör API'si kullanılmaz — tamamen piksel analizi.
    """

    def __init__(self):
        self._red_count = 0         # ardışık kırmızı tespit sayacı
        self._green_count = 0       # ardışık yeşil tespit sayacı
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._frame_count = 0

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def process(self, frame) -> TrafficLightResult:
        """
        BGR frame'den trafik ışığı tespit et.

        Args:
            frame: np.ndarray (H, W, 3) BGR veya None

        Returns:
            TrafficLightResult
        """
        if frame is None:
            return TrafficLightResult()

        self._frame_count += 1
        h, w = frame.shape[:2]

        # ── 1. ROI — üst kısım (ışıklar gökyüzüne yakın) ────────────
        roi_h = int(h * TL_ROI_RATIO)
        roi = frame[0:roi_h, :]

        # ── 2. HSV dönüşümü ──────────────────────────────────────────
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ── 3. Kırmızı maske (iki aralık — H=0-10 ve H=170-180) ─────
        mask_r1 = cv2.inRange(hsv,
                              np.array(TL_RED_HSV_LOW1),
                              np.array(TL_RED_HSV_HIGH1))
        mask_r2 = cv2.inRange(hsv,
                              np.array(TL_RED_HSV_LOW2),
                              np.array(TL_RED_HSV_HIGH2))
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)

        # ── 4. Yeşil maske ───────────────────────────────────────────
        green_mask = cv2.inRange(hsv,
                                 np.array(TL_GREEN_HSV_LOW),
                                 np.array(TL_GREEN_HSV_HIGH))

        # ── 5. Morfolojik temizlik ───────────────────────────────────
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self._kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self._kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, self._kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, self._kernel)

        # ── 6. Kontur tespiti ────────────────────────────────────────
        red_contours = self._find_light_contours(red_mask, roi)
        green_contours = self._find_light_contours(green_mask, roi)

        # ── 7. En büyük kontur alanları ──────────────────────────────
        red_best = self._best_contour(red_contours)
        green_best = self._best_contour(green_contours)

        red_area = cv2.contourArea(red_best) if red_best is not None else 0
        green_area = cv2.contourArea(green_best) if green_best is not None else 0

        # ── 8. Karar mantığı ────────────────────────────────────────
        state = "none"
        center = None
        area = 0
        contour_count = 0

        if red_area > TL_MIN_AREA and red_area > green_area * 0.5:
            # Kırmızı baskın
            self._red_count += 1
            self._green_count = 0
            state = "red"
            area = int(red_area)
            center = self._contour_center(red_best)
            contour_count = len(red_contours)

        elif green_area > TL_MIN_AREA:
            # Yeşil baskın
            self._green_count += 1
            self._red_count = 0
            state = "green"
            area = int(green_area)
            center = self._contour_center(green_best)
            contour_count = len(green_contours)

        else:
            # Hiçbiri tespit edilmedi → sayaçları yavaşça azalt
            self._red_count = max(0, self._red_count - 1)
            self._green_count = max(0, self._green_count - 1)

        confirmed = self._red_count >= TL_CONFIRM_FRAMES

        # ── 9. Debug overlay ────────────────────────────────────────
        overlay = self._draw_overlay(
            frame.copy(), roi_h,
            red_contours, green_contours,
            red_best, green_best,
            state, confirmed,
        )

        return TrafficLightResult(
            state=state,
            confirmed=confirmed,
            area=area,
            center=center,
            contour_count=contour_count,
            overlay_frame=overlay,
            red_mask=red_mask,
            green_mask=green_mask,
            roi_h=roi_h,
            red_contours=red_contours,
            green_contours=green_contours,
        )

    # =================================================================
    #  KONTUR FİLTRESİ
    # =================================================================

    def _find_light_contours(self, mask, roi):
        """
        Maskeden trafik ışığına benzeyen konturları bul.
        Filtreler: alan aralığı + dairesellik.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        roi_h, roi_w = mask.shape[:2]
        max_bbox_w = max(TL_MIN_BBOX_PX, int(roi_w * TL_MAX_BBOX_RATIO))
        max_bbox_h = max(TL_MIN_BBOX_PX, int(roi_h * TL_MAX_BBOX_RATIO))
        x_margin = int(roi_w * TL_CENTER_MARGIN_RATIO)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < TL_MIN_AREA or area > TL_MAX_AREA:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw < TL_MIN_BBOX_PX or bh < TL_MIN_BBOX_PX:
                continue
            if bw > max_bbox_w or bh > max_bbox_h:
                continue
            if x < x_margin or x + bw > roi_w - x_margin:
                continue

            aspect = bw / float(bh)
            if aspect < TL_MIN_ASPECT_RATIO or aspect > TL_MAX_ASPECT_RATIO:
                continue

            extent = area / float(bw * bh)
            if extent < TL_MIN_EXTENT or extent > TL_MAX_EXTENT:
                continue

            # Dairesellik: 4π × alan / çevre²  (daire = 1.0)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter < 1.0:
                continue
            circularity = (4.0 * math.pi * area) / (perimeter * perimeter)
            if circularity < TL_MIN_CIRCULARITY:
                continue

            if not self._has_dark_light_housing(cnt, mask, roi):
                continue

            valid.append(cnt)

        return valid

    @staticmethod
    def _has_dark_light_housing(cnt, mask, roi):
        """Check for the dark casing that surrounds CARLA traffic-light lamps."""
        roi_h, roi_w = mask.shape[:2]
        x, y, bw, bh = cv2.boundingRect(cnt)

        pad_x = max(4, int(bw * TL_CONTEXT_PAD_RATIO))
        pad_y = max(6, int(bh * TL_CONTEXT_PAD_RATIO))
        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(roi_w, x + bw + pad_x)
        y2 = min(roi_h, y + bh + pad_y)

        if x2 <= x1 or y2 <= y1:
            return False

        context = roi[y1:y2, x1:x2]
        context_mask = mask[y1:y2, x1:x2]
        gray = cv2.cvtColor(context, cv2.COLOR_BGR2GRAY)

        colored = context_mask > 0
        dark = (gray < 85) & (~colored)
        context_pixels = max(1, dark.size - int(np.count_nonzero(colored)))
        dark_ratio = float(np.count_nonzero(dark)) / context_pixels

        return dark_ratio >= TL_MIN_DARK_CONTEXT

    @staticmethod
    def _best_contour(contours):
        """En büyük alanlı konturu döndür."""
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    @staticmethod
    def _contour_center(cnt):
        """Kontur merkez koordinatı."""
        if cnt is None:
            return None
        M = cv2.moments(cnt)
        if M["m00"] < 1:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)

    # =================================================================
    #  DEBUG ÇİZİMİ
    # =================================================================

    def _draw_overlay(self, frame, roi_h,
                      red_contours, green_contours,
                      red_best, green_best,
                      state, confirmed):
        """Tespit edilen konturları ve durumu frame üzerine çiz."""

        # ROI sınırını çiz
        cv2.line(frame, (0, roi_h), (frame.shape[1], roi_h),
                 (100, 100, 100), 1)

        # Kırmızı konturları çiz
        for cnt in red_contours:
            cv2.drawContours(frame, [cnt], -1, (80, 90, 200), 1)
        if red_best is not None:
            cv2.drawContours(frame, [red_best], -1, (80, 90, 200), 2)
            center = self._contour_center(red_best)
            if center:
                cv2.circle(frame, center, 5, (80, 90, 200), -1)

        # Yeşil konturları çiz
        for cnt in green_contours:
            cv2.drawContours(frame, [cnt], -1, (80, 190, 100), 1)
        if green_best is not None:
            cv2.drawContours(frame, [green_best], -1, (80, 190, 100), 2)
            center = self._contour_center(green_best)
            if center:
                cv2.circle(frame, center, 5, (80, 190, 100), -1)

        # Durum etiketi
        h, w = frame.shape[:2]
        if state == "red":
            label = "RED" + (" STOP" if confirmed else "")
            color = (80, 90, 200)
        elif state == "green":
            label = "GREEN"
            color = (80, 190, 100)
        else:
            label = ""
            color = (140, 140, 150)

        if label:
            cv2.putText(frame, label, (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        return frame
