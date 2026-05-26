"""
controllers/lane_controller.py — OpenCV tabanlı şerit tespit ve PID kontrolcüsü

Pipeline (her frame):
  1. RGB → HLS renk uzayı dönüşümü
  2. Beyaz şerit maskesi (yüksek Lightness, düşük Saturation)
  3. Sarı şerit maskesi (Hue 15-35, orta-yüksek Saturation)
  4. Canny kenar tespiti
  5. Maskelerin birleşimi (beyaz | sarı | canny)
  6. ROI (Region of Interest) — alt yarı kırpma
  7. Perspective Warp → kuşbakışı (bird's-eye view)
  8. Sliding Window → sol/sağ şerit piksel tespiti
  9. 2. derece polinom fit (ax² + bx + c)
 10. Lane center offset hesaplama
 11. PID kontrolcü → direksiyon açısı
 12. Hız kontrolü → throttle / brake

Kullanım:
    ctrl = LaneController()
    ...
    control, is_valid = ctrl.process_frame(bgr_frame, current_speed_ms)
    # control: carla.VehicleControl
    # is_valid: bool (şerit bulundu mu?)
"""

import math
import cv2
import numpy as np
import carla

from config import (
    LANE_CAM_W, LANE_CAM_H,
    WHITE_L_THRESH, WHITE_S_THRESH,
    YELLOW_H_THRESH, YELLOW_S_THRESH, YELLOW_L_THRESH,
    CANNY_LOW, CANNY_HIGH,
    WARP_SRC, WARP_DST,
    SW_NWINDOWS, SW_MARGIN, SW_MINPIX,
    LANE_KP, LANE_KI, LANE_KD,
    LANE_SPEED_MS, LANE_THROTTLE, LANE_BRAKE_KP,
    LANE_MIN_PIXELS,
    LANE_DEBUG_WIN, LANE_DEBUG_W, LANE_DEBUG_H,
)


class LaneController:
    """
    Kamera görüntüsünden şerit tespiti yaparak direksiyon kontrolü üretir.

    Attributes:
        lane_valid (bool): Son frame'de şerit bulundu mu
        offset_px (float): Şerit merkezinden sapma (piksel)
        left_fit, right_fit: Son polinom katsayıları
    """

    def __init__(self):
        # ── Perspective warp matrisleri (bir kere hesapla) ────────────
        src = np.float32(WARP_SRC)
        dst = np.float32(WARP_DST)
        self._M = cv2.getPerspectiveTransform(src, dst)
        self._M_inv = cv2.getPerspectiveTransform(dst, src)

        # ── PID state ────────────────────────────────────────────────
        self._integral = 0.0
        self._prev_error = 0.0

        # ── Sliding window son katsayılar (smoothing) ────────────────
        self.left_fit = None
        self.right_fit = None
        self.lane_valid = False
        self.offset_px = 0.0

        # ── Debug overlay frame'i ────────────────────────────────────
        self._debug_frame = None
        self._warped_debug = None

        # ── Sayaçlar ─────────────────────────────────────────────────
        self._frame_count = 0

        # ── Debug penceresi ──────────────────────────────────────────
        if LANE_DEBUG_WIN:
            cv2.namedWindow("Lane Debug", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Lane Debug", LANE_DEBUG_W, LANE_DEBUG_H)
            cv2.moveWindow("Lane Debug", 10, 10)

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def process_frame(self, frame, speed_ms=0.0):
        """
        Tek bir kamera frame'ini işleyip araç kontrolü döndürür.

        Args:
            frame: np.ndarray BGR (H, W, 3)
            speed_ms: mevcut hız (m/s)

        Returns:
            (carla.VehicleControl, bool)
            bool = şerit geçerli mi (True) yoksa kayıp mı (False)
        """
        if frame is None:
            return self._default_control(), False

        self._frame_count += 1

        # ── 1. Renk filtresi (HLS) ──────────────────────────────────
        binary = self._color_threshold(frame)

        # ── 2. Canny kenar tespiti ───────────────────────────────────
        edges = self._canny_edges(frame)

        # ── 3. Birleşik maske ────────────────────────────────────────
        combined = cv2.bitwise_or(binary, edges)

        # ── 4. ROI — sadece alt yarı (yol bölgesi) ──────────────────
        roi = self._apply_roi(combined)

        # ── 5. Perspective Warp → kuşbakışı ─────────────────────────
        warped = cv2.warpPerspective(roi, self._M,
                                     (LANE_CAM_W, LANE_CAM_H))

        # ── 6. Sliding Window → şerit pikselleri ────────────────────
        left_fit, right_fit, left_pts, right_pts, sw_img = \
            self._sliding_window(warped)

        # ── 7. Geçerlilik kontrolü ───────────────────────────────────
        left_ok = left_pts is not None and len(left_pts) > LANE_MIN_PIXELS
        right_ok = right_pts is not None and len(right_pts) > LANE_MIN_PIXELS

        if left_ok and right_ok:
            self.left_fit = left_fit
            self.right_fit = right_fit
            self.lane_valid = True
        elif left_ok and self.right_fit is not None:
            # Sadece sol şerit bulundu — sağı önceki frame'den kullan
            self.left_fit = left_fit
            self.lane_valid = True
        elif right_ok and self.left_fit is not None:
            # Sadece sağ şerit bulundu — solu önceki frame'den kullan
            self.right_fit = right_fit
            self.lane_valid = True
        elif self.left_fit is not None and self.right_fit is not None:
            # Her iki şerit de kayıp ama önceki fit var → devam et
            self.lane_valid = True
        else:
            self.lane_valid = False

        # ── 8. Offset hesapla ────────────────────────────────────────
        if self.lane_valid and self.left_fit is not None and self.right_fit is not None:
            self.offset_px = self._compute_offset(
                self.left_fit, self.right_fit, LANE_CAM_H
            )
        else:
            self.offset_px = 0.0

        # ── 9. PID → steer ──────────────────────────────────────────
        steer = self._pid_steer(self.offset_px)

        # ── 10. Hız kontrolü → throttle / brake ─────────────────────
        throttle, brake = self._speed_control(speed_ms)

        # ── 11. Debug overlay ────────────────────────────────────────
        if LANE_DEBUG_WIN:
            self._build_debug(frame, warped, sw_img, steer, speed_ms)

        # ── 12. VehicleControl oluştur ───────────────────────────────
        ctrl = carla.VehicleControl(
            throttle=float(throttle),
            steer=float(steer),
            brake=float(brake),
            hand_brake=False,
            manual_gear_shift=False,
        )

        return ctrl, self.lane_valid

    def show_debug(self):
        """Debug penceresini güncelle. Simülasyon loop'undan çağrılır."""
        if not LANE_DEBUG_WIN or self._debug_frame is None:
            return True
        cv2.imshow("Lane Debug", self._debug_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            return False
        return True

    def destroy(self):
        """OpenCV pencerelerini kapat."""
        if LANE_DEBUG_WIN:
            try:
                cv2.destroyWindow("Lane Debug")
            except Exception:
                pass

    # =================================================================
    #  STEP 1 — HLS RENK FİLTRESİ
    # =================================================================

    def _color_threshold(self, frame):
        """
        BGR → HLS dönüşümü, beyaz ve sarı şerit maskeleri.

        Beyaz şerit  : L kanalı yüksek (parlak), S kanalı düşük
        Sarı şerit   : H kanalı 15-35, S kanalı yüksek, L orta
        """
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        h, l, s = hls[:, :, 0], hls[:, :, 1], hls[:, :, 2]

        # Beyaz maske
        white = np.zeros_like(l, dtype=np.uint8)
        white[
            (l >= WHITE_L_THRESH[0]) & (l <= WHITE_L_THRESH[1]) &
            (s >= WHITE_S_THRESH[0]) & (s <= WHITE_S_THRESH[1])
        ] = 255

        # Sarı maske
        yellow = np.zeros_like(l, dtype=np.uint8)
        yellow[
            (h >= YELLOW_H_THRESH[0]) & (h <= YELLOW_H_THRESH[1]) &
            (s >= YELLOW_S_THRESH[0]) & (s <= YELLOW_S_THRESH[1]) &
            (l >= YELLOW_L_THRESH[0]) & (l <= YELLOW_L_THRESH[1])
        ] = 255

        return cv2.bitwise_or(white, yellow)

    # =================================================================
    #  STEP 2 — CANNY KENAR TESPİTİ
    # =================================================================

    def _canny_edges(self, frame):
        """Grayscale → GaussianBlur → Canny."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
        return edges

    # =================================================================
    #  STEP 3 — ROI (İLGİ BÖLGESİ)
    # =================================================================

    def _apply_roi(self, binary):
        """Görüntünün sadece alt yarısını (yol bölgesi) maskele."""
        h, w = binary.shape[:2]
        mask = np.zeros_like(binary)

        # Trapez şeklinde ROI — yolun görüldüğü alan
        polygon = np.array([[
            (0, h),            # sol alt
            (0, int(h * 0.5)), # sol üst (yarı yükseklik)
            (w, int(h * 0.5)), # sağ üst
            (w, h),            # sağ alt
        ]], dtype=np.int32)

        cv2.fillPoly(mask, polygon, 255)
        return cv2.bitwise_and(binary, mask)

    # =================================================================
    #  STEP 4 — SLIDING WINDOW
    # =================================================================

    def _sliding_window(self, warped):
        """
        Histogram tabanlı kayan pencere ile sol/sağ şerit piksellerini bul.

        Returns:
            left_fit   : np.array [a, b, c] veya None
            right_fit  : np.array [a, b, c] veya None
            left_pts   : (y_coords, x_coords) veya None
            right_pts  : (y_coords, x_coords) veya None
            out_img    : debug görselleştirme (3-kanallı)
        """
        h, w = warped.shape[:2]

        # Histogram (alt yarı)
        histogram = np.sum(warped[h // 2:, :], axis=0)
        midpoint = w // 2

        # Sol ve sağ tepe noktaları
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        # Debug görüntüsü
        out_img = np.dstack((warped, warped, warped))

        # Pencere yüksekliği
        win_h = h // SW_NWINDOWS

        # Tüm beyaz piksellerin koordinatları
        nonzero = warped.nonzero()
        nz_y = np.array(nonzero[0])
        nz_x = np.array(nonzero[1])

        # Mevcut merkez pozisyonları
        left_current = left_base
        right_current = right_base

        # Şerit piksel indeksleri
        left_lane_inds = []
        right_lane_inds = []

        for win in range(SW_NWINDOWS):
            # Pencere sınırları
            y_low = h - (win + 1) * win_h
            y_high = h - win * win_h

            xl_low = left_current - SW_MARGIN
            xl_high = left_current + SW_MARGIN
            xr_low = right_current - SW_MARGIN
            xr_high = right_current + SW_MARGIN

            # Debug: pencereleri çiz
            cv2.rectangle(out_img, (xl_low, y_low), (xl_high, y_high),
                          (0, 255, 0), 2)
            cv2.rectangle(out_img, (xr_low, y_low), (xr_high, y_high),
                          (0, 255, 0), 2)

            # Pencere içindeki pikselleri bul
            good_left = (
                (nz_y >= y_low) & (nz_y < y_high) &
                (nz_x >= xl_low) & (nz_x < xl_high)
            ).nonzero()[0]

            good_right = (
                (nz_y >= y_low) & (nz_y < y_high) &
                (nz_x >= xr_low) & (nz_x < xr_high)
            ).nonzero()[0]

            left_lane_inds.append(good_left)
            right_lane_inds.append(good_right)

            # Yeterli piksel varsa merkezi güncelle
            if len(good_left) > SW_MINPIX:
                left_current = int(np.mean(nz_x[good_left]))
            if len(good_right) > SW_MINPIX:
                right_current = int(np.mean(nz_x[good_right]))

        # İndeksleri birleştir
        left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else np.array([])
        right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else np.array([])

        # Sol şerit pikselleri
        left_pts = None
        left_fit = None
        if len(left_lane_inds) > 0:
            lx = nz_x[left_lane_inds]
            ly = nz_y[left_lane_inds]
            left_pts = (ly, lx)
            if len(lx) > 50:  # Minimum polinom fit için
                try:
                    left_fit = np.polyfit(ly, lx, 2)
                except (np.linalg.LinAlgError, ValueError):
                    left_fit = None
            # Debug: pikselleri renklendir
            out_img[ly, lx] = [255, 0, 0]  # Mavi

        # Sağ şerit pikselleri
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
            out_img[ry, rx] = [0, 0, 255]  # Kırmızı

        # Polinom eğrilerini çiz
        if left_fit is not None:
            plot_y = np.linspace(0, h - 1, h)
            plot_x = np.clip(
                (left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]).astype(int),
                0, w - 1
            )
            pts = np.column_stack((plot_x, plot_y.astype(int)))
            cv2.polylines(out_img, [pts], False, (255, 255, 0), 2)

        if right_fit is not None:
            plot_y = np.linspace(0, h - 1, h)
            plot_x = np.clip(
                (right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]).astype(int),
                0, w - 1
            )
            pts = np.column_stack((plot_x, plot_y.astype(int)))
            cv2.polylines(out_img, [pts], False, (255, 255, 0), 2)

        return left_fit, right_fit, left_pts, right_pts, out_img

    # =================================================================
    #  STEP 5 — OFFSET HESAPLAMA
    # =================================================================

    def _compute_offset(self, left_fit, right_fit, h):
        """
        Şerit merkezinden sapma (piksel cinsinden).
        Pozitif = araç sağda, Negatif = araç solda.
        """
        # Görüntünün en alt satırında (aracın konumu) şerit pozisyonları
        y_eval = h - 1
        left_x = left_fit[0] * y_eval**2 + left_fit[1] * y_eval + left_fit[2]
        right_x = right_fit[0] * y_eval**2 + right_fit[1] * y_eval + right_fit[2]

        # Şerit merkezi
        lane_center = (left_x + right_x) / 2.0
        # Görüntü merkezi (= araç merkezi)
        image_center = LANE_CAM_W / 2.0

        # Offset: pozitif = araç sağa kaymış → sola dönmeli
        return lane_center - image_center

    # =================================================================
    #  STEP 6 — PID DİREKSİYON KONTROLÜ
    # =================================================================

    def _pid_steer(self, offset):
        """
        PID kontrolcü: piksel offset → direksiyon açısı [-1, 1].

        P: Anlık hataya oransal tepki
        I: Birikmiş hatayı düzeltme (drift)
        D: Hata değişim hızına tepki (salınımı azaltır)
        """
        error = offset

        # İntegral (birikim) — anti-windup
        self._integral += error
        self._integral = max(-5000.0, min(5000.0, self._integral))

        # Türev
        derivative = error - self._prev_error
        self._prev_error = error

        # PID çıkışı
        steer = (LANE_KP * error +
                 LANE_KI * self._integral +
                 LANE_KD * derivative)

        # Sınırlama [-1, 1]
        return max(-1.0, min(1.0, steer))

    # =================================================================
    #  STEP 7 — HIZ KONTROLÜ
    # =================================================================

    def _speed_control(self, speed_ms):
        """Hedef hıza göre throttle / brake hesapla."""
        err = LANE_SPEED_MS - speed_ms
        if err > 0:
            throttle = min(LANE_THROTTLE + 0.3 * (err / LANE_SPEED_MS), 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(LANE_BRAKE_KP * abs(err) / LANE_SPEED_MS, 0.5)
        return throttle, brake

    # =================================================================
    #  DEBUG OVERLAY
    # =================================================================

    def _build_debug(self, frame, warped, sw_img, steer, speed_ms):
        """
        Debug penceresi için birleşik görüntü oluştur.

        Layout:
        ┌──────────────┬──────────────┐
        │  Orijinal +  │  Warped +    │
        │  Şerit Over. │  Sliding Win │
        └──────────────┴──────────────┘
        """
        h, w = frame.shape[:2]

        # ── Sol panel: orijinal + overlay ────────────────────────────
        overlay = frame.copy()

        # Warp kaynak trapezini çiz
        src_pts = np.array(WARP_SRC, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(overlay, [src_pts], True, (0, 255, 255), 2)

        # Şerit alanını boyama (ters warp ile orijinale yansıt)
        if self.left_fit is not None and self.right_fit is not None:
            lane_overlay = np.zeros_like(frame)
            plot_y = np.linspace(0, h - 1, h)
            left_x = np.clip(
                (self.left_fit[0] * plot_y**2 +
                 self.left_fit[1] * plot_y +
                 self.left_fit[2]).astype(int),
                0, w - 1
            )
            right_x = np.clip(
                (self.right_fit[0] * plot_y**2 +
                 self.right_fit[1] * plot_y +
                 self.right_fit[2]).astype(int),
                0, w - 1
            )

            # Poligon doldurma (yeşil şerit alanı)
            pts_left = np.column_stack((left_x, plot_y.astype(int)))
            pts_right = np.flipud(np.column_stack((right_x, plot_y.astype(int))))
            fill_pts = np.vstack((pts_left, pts_right))

            cv2.fillPoly(lane_overlay, [fill_pts], (0, 180, 0))

            # Ters warp → orijinal perspektife dönüştür
            unwarped = cv2.warpPerspective(
                lane_overlay, self._M_inv, (w, h)
            )
            overlay = cv2.addWeighted(overlay, 1.0, unwarped, 0.4, 0)

        # HUD bilgileri
        status_txt = "LANE OK" if self.lane_valid else "LANE LOST"
        status_col = (0, 255, 0) if self.lane_valid else (0, 0, 255)
        cv2.putText(overlay, status_txt, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_col, 2)
        cv2.putText(overlay, f"Offset: {self.offset_px:+.0f}px", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(overlay, f"Steer: {steer:+.3f}", (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(overlay, f"Speed: {speed_ms * 3.6:.1f} km/h", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Direksiyon göstergesi (yatay çubuk)
        bar_cx = w // 2
        bar_y = h - 30
        bar_len = int(steer * (w // 4))
        cv2.line(overlay, (bar_cx, bar_y), (bar_cx + bar_len, bar_y),
                 (0, 200, 255), 4)
        cv2.circle(overlay, (bar_cx, bar_y), 6, (255, 255, 255), -1)

        # ── Sağ panel: sliding window sonucu ─────────────────────────
        # Her iki paneli aynı boyuta getir
        half_w = w // 2
        left_panel = cv2.resize(overlay, (half_w, h // 2))
        right_panel = cv2.resize(sw_img, (half_w, h // 2))

        # Birleştir
        self._debug_frame = np.hstack((left_panel, right_panel))

    # =================================================================
    #  FALLBACK KONTROL
    # =================================================================

    @staticmethod
    def _default_control():
        """Şerit bulunamadığında güvenli kontrol: düşük gaz, düz direksiyon."""
        return carla.VehicleControl(
            throttle=0.3, steer=0.0, brake=0.0,
            hand_brake=False, manual_gear_shift=False,
        )
