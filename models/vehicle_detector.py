"""
models/vehicle_detector.py — OpenCV Tabanlı Araç Tespiti

Ön kamera görüntüsünden NPC araçları tespit eder.

Pipeline:
  1. ROI — Görüntünün alt kısmı (araçlar yol seviyesinde)
  2. Gri tonlama + Gaussian blur (gürültü azaltma)
  3. Canny kenar tespiti
  4. Morfolojik genişletme (kapalı alan oluştur)
  5. Kontur tespiti + alan/aspect ratio filtresi
  6. Perspektif geometri ile mesafe tahmini
  7. Çok-frame doğrulama (false positive azaltma)

Kullanım:
    detector = VehicleDetector()
    result = detector.process(bgr_frame)
    if result.detected:
        print(f"En yakın araç: {result.closest_distance_m:.1f}m")
"""

from dataclasses import dataclass, field
from typing import Optional, List, Tuple

import cv2
import numpy as np

from config import (
    VD_ROI_TOP_RATIO, VD_MIN_AREA, VD_MAX_AREA,
    VD_MIN_ASPECT, VD_MAX_ASPECT,
    VD_REAL_WIDTH_M, VD_FOCAL_LENGTH_PX,
    VD_SAFE_DIST_M, VD_CRITICAL_DIST_M, VD_EMERGENCY_DIST_M,
    VD_CONFIRM_FRAMES, VD_MORPH_KERNEL, VD_CANNY_LOW, VD_CANNY_HIGH,
    VD_DILATE_ITER, LANE_CAM_W, LANE_CAM_H,
)


# =====================================================================
#  SONUÇ VERİ YAPISI
# =====================================================================

@dataclass
class VehicleDetection:
    """Tek araç tespiti — bounding box + mesafe tahmini."""
    x: int = 0          # bbox sol kenar
    y: int = 0          # bbox üst kenar
    w: int = 0          # bbox genişliği
    h: int = 0          # bbox yüksekliği
    distance_m: float = 0.0  # tahmini mesafe
    confidence: float = 0.0  # tespit güveni (0-1)

    @property
    def cx(self) -> int:
        """Bounding box yatay merkezi."""
        return self.x + self.w // 2

    @property
    def cy(self) -> int:
        """Bounding box dikey merkezi."""
        return self.y + self.h // 2

    @property
    def is_close(self) -> bool:
        return self.distance_m < VD_SAFE_DIST_M

    @property
    def is_critical(self) -> bool:
        return self.distance_m < VD_CRITICAL_DIST_M

    @property
    def is_emergency(self) -> bool:
        return self.distance_m < VD_EMERGENCY_DIST_M


@dataclass
class VehicleDetectionResult:
    """Bir frame'in tam araç tespit sonucu."""
    detected: bool = False
    vehicles: List[VehicleDetection] = field(default_factory=list)
    closest_distance_m: float = float("inf")
    vehicle_count: int = 0
    overlay_frame: Optional[np.ndarray] = field(default=None, repr=False)
    roi_y: int = 0            # ROI başlangıç Y koordinatı (orijinal frame)

    # ── Durum özeti ──────────────────────────────────────────────────
    @property
    def status(self) -> str:
        """İnsan okunabilir durum: FREE / FOLLOWING / BRAKING / EMERGENCY"""
        if not self.detected:
            return "FREE"
        d = self.closest_distance_m
        if d < VD_EMERGENCY_DIST_M:
            return "EMERGENCY"
        if d < VD_CRITICAL_DIST_M:
            return "BRAKING"
        if d < VD_SAFE_DIST_M:
            return "FOLLOWING"
        return "FREE"

    @property
    def closest(self) -> Optional[VehicleDetection]:
        """En yakın tespit edilen araç."""
        if not self.vehicles:
            return None
        return min(self.vehicles, key=lambda v: v.distance_m)


# =====================================================================
#  ANA SINIF
# =====================================================================

class VehicleDetector:
    """
    OpenCV tabanlı araç tespiti modülü.

    Ön kamera görüntüsünde diğer araçları tespit eder,
    mesafe tahmini yapar ve sonuçları overlay ile raporlar.
    """

    # Durum göstergesi renkleri (BGR)
    _COLOR_FREE      = (90, 200, 90)    # yeşil
    _COLOR_FOLLOWING = (30, 200, 220)   # sarı
    _COLOR_BRAKING   = (30, 100, 220)   # turuncu
    _COLOR_EMERGENCY = (30, 30, 200)    # kırmızı

    def __init__(self):
        # Morfolojik kernel
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (VD_MORPH_KERNEL, VD_MORPH_KERNEL)
        )
        # Arka plan çıkarıcı (hareket eden nesneler için)
        self._bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=120, varThreshold=40, detectShadows=False
        )
        # Frame doğrulama sayacı: {bbox_key: count}
        self._confirm_counts: dict[str, int] = {}
        self._frame_count = 0
        self._last_boxes: List[Tuple] = []  # önceki frame kutuları

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def process(self, frame: Optional[np.ndarray]) -> VehicleDetectionResult:
        """
        BGR kamera frame'inden araç tespit et.

        Args:
            frame: np.ndarray (H, W, 3) BGR veya None

        Returns:
            VehicleDetectionResult
        """
        if frame is None:
            return VehicleDetectionResult()

        self._frame_count += 1
        h, w = frame.shape[:2]
        roi_y = int(h * VD_ROI_TOP_RATIO)

        # ── 1. ROI kırp ─────────────────────────────────────────────
        roi = frame[roi_y:h, :]

        # ── 2. Ön işleme ─────────────────────────────────────────────
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # ── 3. Canny kenar tespiti ───────────────────────────────────
        edges = cv2.Canny(blurred, VD_CANNY_LOW, VD_CANNY_HIGH)

        # ── 4. Arka plan çıkarma (hareket maskesi) ───────────────────
        fg_mask = self._bg_subtractor.apply(roi)
        fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)[1]

        # ── 5. Kenar + hareket maskesi birleştir ─────────────────────
        combined = cv2.bitwise_or(edges, fg_mask)

        # ── 6. Morfolojik kapatma — aralıkları doldur ────────────────
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, self._kernel)
        combined = cv2.dilate(combined, self._kernel, iterations=VD_DILATE_ITER)

        # ── 7. Kontur tespiti ────────────────────────────────────────
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        # ── 8. Kontur filtrele → araç adayları ───────────────────────
        vehicles = self._filter_contours(contours, roi, roi_y, w, h)

        # ── 9. Frame doğrulama ───────────────────────────────────────
        vehicles = self._validate(vehicles)

        # ── 10. Sonuç ────────────────────────────────────────────────
        detected = len(vehicles) > 0
        closest_dist = min((v.distance_m for v in vehicles), default=float("inf"))

        overlay = self._draw_overlay(frame.copy(), vehicles, roi_y, h, w)

        result = VehicleDetectionResult(
            detected=detected,
            vehicles=vehicles,
            closest_distance_m=closest_dist,
            vehicle_count=len(vehicles),
            overlay_frame=overlay,
            roi_y=roi_y,
        )

        self._last_boxes = [(v.x, v.y, v.w, v.h) for v in vehicles]
        return result

    def reset(self):
        """Arka plan modeli ve doğrulama sayaçlarını sıfırla."""
        self._bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=120, varThreshold=40, detectShadows=False
        )
        self._confirm_counts.clear()
        self._frame_count = 0

    # =================================================================
    #  KONTUR FİLTRE
    # =================================================================

    def _filter_contours(self, contours, roi, roi_y: int,
                          frame_w: int, frame_h: int) -> List[VehicleDetection]:
        """
        Ham konturları filtrele, araç adaylarına dönüştür.
        """
        roi_h = roi.shape[0]
        results = []

        # Araçlar genellikle görüntünün alt yarısında ve ortada
        center_x = frame_w // 2
        min_x = int(frame_w * 0.05)    # Kenar gürültüsünü atla
        max_x = int(frame_w * 0.95)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < VD_MIN_AREA or area > VD_MAX_AREA:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)

            # Kenar filtresi
            if x < min_x or (x + bw) > max_x:
                continue

            # Aspect ratio filtresi (araçlar genellikle yatay uzun)
            if bh < 1:
                continue
            aspect = bw / float(bh)
            if aspect < VD_MIN_ASPECT or aspect > VD_MAX_ASPECT:
                continue

            # Mesafe tahmini — perspektif geometri
            # f * W / pixel_width = D
            distance_m = self._estimate_distance(bw)

            # Çok uzaktaki araçları görmezden gel (gürültü)
            if distance_m > 60.0:
                continue

            # Güven skoru: kutu merkezi ne kadar ortada + boyut uyumu
            box_cx = x + bw // 2
            center_score = 1.0 - abs(box_cx - center_x) / (frame_w / 2)
            size_score = min(area / VD_MIN_AREA, 5.0) / 5.0
            confidence = (center_score * 0.4 + size_score * 0.6)

            # ROI orijinal frame koordinatına dönüştür
            abs_y = roi_y + y

            results.append(VehicleDetection(
                x=x, y=abs_y, w=bw, h=bh,
                distance_m=distance_m,
                confidence=confidence,
            ))

        # Mesafeye göre sırala (en yakın önce)
        results.sort(key=lambda v: v.distance_m)

        # En fazla 5 araç raporla
        return results[:5]

    @staticmethod
    def _estimate_distance(pixel_width: int) -> float:
        """
        Perspektif geometri ile araç mesafesini tahmin et.

        distance = focal_length * real_width / pixel_width

        Args:
            pixel_width: Görüntüdeki araç genişliği (piksel)

        Returns:
            Tahmini mesafe (metre)
        """
        if pixel_width < 1:
            return float("inf")
        return (VD_FOCAL_LENGTH_PX * VD_REAL_WIDTH_M) / pixel_width

    # =================================================================
    #  ÇOKLU-FRAME DOĞRULAMA
    # =================================================================

    def _validate(self, vehicles: List[VehicleDetection]) -> List[VehicleDetection]:
        """
        Tespit edilen araçları frame sayısına göre doğrula.
        Önceki frame'deki kutuyla eşleşenlerin sayacını artır.
        """
        if VD_CONFIRM_FRAMES <= 1:
            return vehicles

        validated = []
        new_counts: dict[str, int] = {}

        for v in vehicles:
            key = self._bbox_key(v.x, v.y, v.w, v.h)
            prev_count = 0

            # Önceki frame'deki yakın kutuyla eşleştir
            for pk, pc in self._confirm_counts.items():
                if self._boxes_overlap(key, pk, tolerance=40):
                    prev_count = max(prev_count, pc)
                    break

            new_count = prev_count + 1
            new_counts[key] = new_count

            if new_count >= VD_CONFIRM_FRAMES:
                validated.append(v)

        self._confirm_counts = new_counts
        return validated

    @staticmethod
    def _bbox_key(x: int, y: int, w: int, h: int, grid: int = 40) -> str:
        """Bounding box'ı ızgara hücresi anahtarına dönüştür."""
        return f"{x//grid},{y//grid},{(x+w)//grid},{(y+h)//grid}"

    @staticmethod
    def _boxes_overlap(key1: str, key2: str, tolerance: int = 40) -> bool:
        """İki ızgara anahtarının yakınlığını kontrol et."""
        try:
            parts1 = [int(p) for p in key1.split(",")]
            parts2 = [int(p) for p in key2.split(",")]
            return all(abs(a - b) <= 1 for a, b in zip(parts1, parts2))
        except Exception:
            return False

    # =================================================================
    #  OVERLAY ÇİZİMİ
    # =================================================================

    def _draw_overlay(self, frame: np.ndarray,
                       vehicles: List[VehicleDetection],
                       roi_y: int, h: int, w: int) -> np.ndarray:
        """
        Tespit sonuçlarını frame üzerine çiz:
        - ROI sınır çizgisi
        - Bounding box'lar (renge göre durum)
        - Mesafe etiketi
        - Durum banner'ı
        """
        # ROI sınır çizgisi
        cv2.line(frame, (0, roi_y), (w, roi_y), (80, 80, 80), 1)

        for v in vehicles:
            color = self._detection_color(v)
            # Bounding box
            cv2.rectangle(frame, (v.x, v.y), (v.x + v.w, v.y + v.h), color, 2)

            # Mesafe etiketi
            label = f"{v.distance_m:.1f}m"
            lx, ly = v.x, max(v.y - 8, roi_y + 4)
            cv2.putText(frame, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

            # Merkez noktası
            cv2.circle(frame, (v.cx, v.cy), 3, color, -1)

        # Durum banner
        if vehicles:
            closest = min(vehicles, key=lambda v: v.distance_m)
            status = self._status_text(closest.distance_m)
            color = self._detection_color(closest)
            cv2.putText(frame, f"VEH {status}", (w - 160, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        return frame

    def _detection_color(self, v: VehicleDetection) -> Tuple[int, int, int]:
        if v.is_emergency:
            return self._COLOR_EMERGENCY
        if v.is_critical:
            return self._COLOR_BRAKING
        if v.is_close:
            return self._COLOR_FOLLOWING
        return self._COLOR_FREE

    @staticmethod
    def _status_text(dist: float) -> str:
        if dist < VD_EMERGENCY_DIST_M:
            return "EMERGENCY"
        if dist < VD_CRITICAL_DIST_M:
            return "BRAKE"
        if dist < VD_SAFE_DIST_M:
            return "FOLLOW"
        return "CLEAR"
