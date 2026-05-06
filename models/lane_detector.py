"""
models/lane_detector.py — OpenCV lane detection pipeline

Pipeline:
  BGR Frame → HLS/HSV Color Threshold + Sobel Gradient
  → Perspective Warp (Bird's Eye) → Sliding Window / Guided Search
  → 2nd-Degree Polynomial Fit → Curvature + Lateral Offset
  → Lane Overlay Drawing

All parameters are sourced from config.py for easy tuning.
"""

from dataclasses import dataclass, field
from collections import deque
from typing import Optional, Tuple

import numpy as np
import cv2

from config import (
    LANE_CAM_W, LANE_CAM_H,
    LANE_SRC, LANE_DST,
    LANE_NWINDOWS, LANE_MARGIN, LANE_MINPIX,
    LANE_WHITE_L_THRESH, LANE_YELLOW_LOWER, LANE_YELLOW_UPPER,
    LANE_SOBEL_LOW, LANE_SOBEL_HIGH,
    LANE_S_THRESH_LOW, LANE_S_THRESH_HIGH,
    LANE_HISTORY_LEN,
)
from utils.logger import log


# ═════════════════════════════════════════════════════════════════════════
#  Data Classes
# ═════════════════════════════════════════════════════════════════════════

@dataclass
class LaneResult:
    """Output of the lane detection pipeline for a single frame."""
    detected: bool = False
    left_fit: Optional[np.ndarray] = None     # [a, b, c] for ax²+bx+c
    right_fit: Optional[np.ndarray] = None
    curvature_m: float = float("inf")         # radius of curvature (meters)
    lateral_offset_m: float = 0.0             # offset from center (+right, -left)
    confidence: float = 0.0                   # 0.0 – 1.0

    # Debug / visualization images
    overlay: Optional[np.ndarray] = None      # annotated original frame
    binary: Optional[np.ndarray] = None       # thresholded binary
    warped: Optional[np.ndarray] = None       # bird's-eye view


# ═════════════════════════════════════════════════════════════════════════
#  Lane Detector
# ═════════════════════════════════════════════════════════════════════════

class LaneDetector:
    """
    Full OpenCV lane detection pipeline.

    Usage:
        detector = LaneDetector()
        result = detector.process(bgr_frame)
        # result.detected, result.lateral_offset_m, result.overlay, ...
    """

    # Approximate meters-per-pixel for CARLA Town04
    YM_PER_PIX = 30.0 / 720    # ~30 m visible ahead per image height
    XM_PER_PIX = 3.7  / 400    # lane width ≈ 3.7 m

    def __init__(self, w: int = LANE_CAM_W, h: int = LANE_CAM_H):
        self.w = w
        self.h = h

        # Perspective transform matrices
        src = np.float32([(x * w, y * h) for x, y in LANE_SRC])
        dst = np.float32([(x * w, y * h) for x, y in LANE_DST])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.M_inv = cv2.getPerspectiveTransform(dst, src)

        # Fit history for smoothing
        self._left_hist: deque = deque(maxlen=LANE_HISTORY_LEN)
        self._right_hist: deque = deque(maxlen=LANE_HISTORY_LEN)

        # Previous fits for guided search
        self._prev_left: Optional[np.ndarray] = None
        self._prev_right: Optional[np.ndarray] = None

        # Consecutive miss counter (resets on detection)
        self._miss_count = 0

        # Offset smoothing state
        self._offset_ema = 0.0

        log("LaneDetector initialized")

    # ─── Main Pipeline ───────────────────────────────────────────────

    def process(self, frame: np.ndarray) -> LaneResult:
        """Run the full pipeline on a BGR frame. Returns LaneResult."""
        if frame is None:
            return LaneResult()

        result = LaneResult()

        # 1. Threshold
        binary = self._threshold(frame)
        result.binary = binary

        # 2. Perspective warp
        warped = cv2.warpPerspective(binary, self.M, (self.w, self.h))
        result.warped = warped

        # 3. Find lane pixels
        if (self._prev_left is not None and self._prev_right is not None
                and self._miss_count < 5):
            lx, ly, rx, ry, conf = self._search_around_poly(warped)
        else:
            lx, ly, rx, ry, conf = self._sliding_window(warped)

        result.confidence = conf

        # 4. Fit polynomials
        if len(lx) > 50 and len(rx) > 50:
            left_fit = np.polyfit(ly, lx, 2)
            right_fit = np.polyfit(ry, rx, 2)

            if self._validate(left_fit, right_fit):
                left_fit = self._smooth(left_fit, self._left_hist)
                right_fit = self._smooth(right_fit, self._right_hist)

                self._prev_left = left_fit
                self._prev_right = right_fit
                self._miss_count = 0

                result.detected = True
                result.left_fit = left_fit
                result.right_fit = right_fit
                result.curvature_m = self._curvature(left_fit, right_fit)
                result.lateral_offset_m = self._offset(left_fit, right_fit)
            else:
                self._miss_count += 1
                self._use_previous(result)
        else:
            self._miss_count += 1
            self._use_previous(result)

        # 5. Draw overlay
        result.overlay = self._draw_overlay(frame, result)
        return result

    # ─── Color + Gradient Threshold ──────────────────────────────────

    def _threshold(self, frame: np.ndarray) -> np.ndarray:
        """Combine color and gradient thresholds → binary image."""
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # White lanes (high lightness)
        l_ch = hls[:, :, 1]
        white = (l_ch > LANE_WHITE_L_THRESH).astype(np.uint8) * 255

        # Yellow lanes
        lower = np.array(LANE_YELLOW_LOWER, dtype=np.uint8)
        upper = np.array(LANE_YELLOW_UPPER, dtype=np.uint8)
        yellow = cv2.inRange(hsv, lower, upper)

        # Sobel-X gradient (vertical edges = lane lines)
        sx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
        abs_sx = np.absolute(sx)
        scaled = np.uint8(255 * abs_sx / (abs_sx.max() + 1e-6))
        sobel = np.zeros_like(gray, dtype=np.uint8)
        sobel[(scaled > LANE_SOBEL_LOW) & (scaled < LANE_SOBEL_HIGH)] = 255

        # S-channel (saturation — good for colored markings)
        s_ch = hls[:, :, 2]
        s_mask = np.zeros_like(gray, dtype=np.uint8)
        s_mask[(s_ch > LANE_S_THRESH_LOW) & (s_ch < LANE_S_THRESH_HIGH)] = 255

        # Combine
        combined = np.zeros_like(gray, dtype=np.uint8)
        combined[(white == 255) |
                 (yellow == 255) |
                 ((sobel == 255) & (s_mask == 255))] = 255

        combined = cv2.GaussianBlur(combined, (5, 5), 0)
        _, combined = cv2.threshold(combined, 127, 255, cv2.THRESH_BINARY)
        return combined

    # ─── Sliding Window Search ───────────────────────────────────────

    def _sliding_window(self, warped: np.ndarray):
        """Full sliding window search from scratch."""
        hist = np.sum(warped[self.h // 2:, :], axis=0)
        mid = self.w // 2
        l_base = int(np.argmax(hist[:mid]))
        r_base = int(np.argmax(hist[mid:])) + mid

        win_h = self.h // LANE_NWINDOWS
        nz = warped.nonzero()
        nz_y, nz_x = np.array(nz[0]), np.array(nz[1])

        l_cur, r_cur = l_base, r_base
        l_inds, r_inds = [], []

        for w in range(LANE_NWINDOWS):
            y_lo = self.h - (w + 1) * win_h
            y_hi = self.h - w * win_h

            good_l = ((nz_y >= y_lo) & (nz_y < y_hi) &
                      (nz_x >= l_cur - LANE_MARGIN) &
                      (nz_x < l_cur + LANE_MARGIN)).nonzero()[0]
            good_r = ((nz_y >= y_lo) & (nz_y < y_hi) &
                      (nz_x >= r_cur - LANE_MARGIN) &
                      (nz_x < r_cur + LANE_MARGIN)).nonzero()[0]

            l_inds.append(good_l)
            r_inds.append(good_r)

            if len(good_l) > LANE_MINPIX:
                l_cur = int(np.mean(nz_x[good_l]))
            if len(good_r) > LANE_MINPIX:
                r_cur = int(np.mean(nz_x[good_r]))

        l_inds = np.concatenate(l_inds) if l_inds else np.array([], dtype=int)
        r_inds = np.concatenate(r_inds) if r_inds else np.array([], dtype=int)

        lx = nz_x[l_inds] if len(l_inds) else np.array([])
        ly = nz_y[l_inds] if len(l_inds) else np.array([])
        rx = nz_x[r_inds] if len(r_inds) else np.array([])
        ry = nz_y[r_inds] if len(r_inds) else np.array([])

        exp = max(self.h * 3, 1)
        conf = (min(len(lx) / exp, 1.0) + min(len(rx) / exp, 1.0)) / 2
        return lx, ly, rx, ry, conf

    # ─── Guided Search (around previous poly) ───────────────────────

    def _search_around_poly(self, warped: np.ndarray):
        """Search near previous polynomial (faster than sliding window)."""
        nz = warped.nonzero()
        nz_y, nz_x = np.array(nz[0]), np.array(nz[1])

        lp = (self._prev_left[0] * nz_y**2 +
              self._prev_left[1] * nz_y + self._prev_left[2])
        rp = (self._prev_right[0] * nz_y**2 +
              self._prev_right[1] * nz_y + self._prev_right[2])

        l_mask = (nz_x > lp - LANE_MARGIN) & (nz_x < lp + LANE_MARGIN)
        r_mask = (nz_x > rp - LANE_MARGIN) & (nz_x < rp + LANE_MARGIN)

        lx, ly = nz_x[l_mask], nz_y[l_mask]
        rx, ry = nz_x[r_mask], nz_y[r_mask]

        if len(lx) < 50 or len(rx) < 50:
            return self._sliding_window(warped)

        exp = max(self.h * 3, 1)
        conf = (min(len(lx) / exp, 1.0) + min(len(rx) / exp, 1.0)) / 2
        return lx, ly, rx, ry, conf

    # ─── Validation ──────────────────────────────────────────────────

    def _validate(self, lf, rf) -> bool:
        """Sanity check: lanes must be roughly parallel, correct width."""
        y_bot = self.h - 1
        lx_bot = lf[0]*y_bot**2 + lf[1]*y_bot + lf[2]
        rx_bot = rf[0]*y_bot**2 + rf[1]*y_bot + rf[2]
        width = rx_bot - lx_bot

        # Reject obviously wrong widths
        if width < 100 or width > 800:
            return False

        lx_top = lf[2]
        rx_top = rf[2]
        top_w = rx_top - lx_top

        # Top width must be positive and not wildly different from bottom
        if top_w < 30 or abs(top_w - width) > width * 1.0:
            return False

        return width > 0

    # ─── Smoothing ───────────────────────────────────────────────────

    def _smooth(self, fit, history: deque) -> np.ndarray:
        """Average current fit with recent history."""
        history.append(fit.copy())
        return np.mean(history, axis=0) if len(history) > 1 else fit

    def _use_previous(self, result: LaneResult):
        """Fall back to cached previous detection."""
        if self._prev_left is not None and self._prev_right is not None:
            result.detected = True
            result.left_fit = self._prev_left
            result.right_fit = self._prev_right
            result.curvature_m = self._curvature(self._prev_left, self._prev_right)
            result.lateral_offset_m = self._offset(self._prev_left, self._prev_right)
            result.confidence *= 0.5

    # ─── Curvature & Offset ──────────────────────────────────────────

    def _curvature(self, lf, rf) -> float:
        """Radius of curvature in meters (average of both lanes)."""
        y_eval = self.h - 1
        la = lf[0] * self.XM_PER_PIX / (self.YM_PER_PIX**2)
        lb = lf[1] * self.XM_PER_PIX / self.YM_PER_PIX
        ra = rf[0] * self.XM_PER_PIX / (self.YM_PER_PIX**2)
        rb = rf[1] * self.XM_PER_PIX / self.YM_PER_PIX
        ym = y_eval * self.YM_PER_PIX
        lr = ((1 + (2*la*ym + lb)**2)**1.5) / (abs(2*la) + 1e-6)
        rr = ((1 + (2*ra*ym + rb)**2)**1.5) / (abs(2*ra) + 1e-6)
        return (lr + rr) / 2.0

    def _offset(self, lf, rf) -> float:
        """Lateral offset from lane center in meters.

        Positive = lane center is RIGHT of image center (car is LEFT of lane)
        Negative = lane center is LEFT of image center (car is RIGHT of lane)
        """
        y = self.h - 1
        lx = lf[0]*y**2 + lf[1]*y + lf[2]
        rx = rf[0]*y**2 + rf[1]*y + rf[2]
        center = (lx + rx) / 2.0
        raw = (center - self.w / 2.0) * self.XM_PER_PIX

        # Clamp extreme values (physical impossibility)
        return max(-2.0, min(2.0, raw))

    # ─── Overlay Drawing ─────────────────────────────────────────────

    def _draw_overlay(self, frame: np.ndarray, result: LaneResult) -> np.ndarray:
        """Draw detected lane lines, polygon, center info onto the original frame."""
        out = frame.copy()

        # ── Always draw the perspective ROI trapezoid ─────────────────
        from config import LANE_SRC
        pts_roi = np.array(
            [(int(x * self.w), int(y * self.h)) for x, y in LANE_SRC],
            dtype=np.int32)
        # Draw ROI with semi-transparent fill
        roi_overlay = out.copy()
        cv2.fillConvexPoly(roi_overlay, pts_roi, (50, 50, 80))
        out = cv2.addWeighted(roi_overlay, 0.3, out, 0.7, 0)
        cv2.polylines(out, [pts_roi], True, (100, 100, 180), 2)

        if not result.detected or result.left_fit is None:
            # Draw "NO LANE" warning
            cv2.putText(out, "NO LANE DETECTED", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            return out

        lf, rf = result.left_fit, result.right_fit

        # Generate lane curves in warped space
        plot_y = np.linspace(0, self.h - 1, self.h)
        lx = lf[0]*plot_y**2 + lf[1]*plot_y + lf[2]
        rx = rf[0]*plot_y**2 + rf[1]*plot_y + rf[2]

        # ── Create lane polygon (green fill) in warped space ─────────
        color_warp = np.zeros((self.h, self.w, 3), dtype=np.uint8)

        pts_l = np.array([np.flipud(np.column_stack((lx, plot_y)))], dtype=np.int32)
        pts_r = np.array([np.column_stack((rx, plot_y))], dtype=np.int32)
        pts = np.hstack((pts_l, pts_r))
        cv2.fillPoly(color_warp, pts, (0, 140, 50))

        # ── Draw thick lane lines in warped space ────────────────────
        # Left lane = CYAN, Right lane = MAGENTA
        left_pts = np.column_stack((lx.astype(int), plot_y.astype(int)))
        right_pts = np.column_stack((rx.astype(int), plot_y.astype(int)))

        cv2.polylines(color_warp, [left_pts], False, (255, 255, 0), 8)    # cyan (BGR)
        cv2.polylines(color_warp, [right_pts], False, (255, 0, 255), 8)   # magenta

        # ── Center line in warped space (YELLOW) ─────────────────────
        cx = ((lx + rx) / 2.0).astype(int)
        center_pts = np.column_stack((cx, plot_y.astype(int)))
        cv2.polylines(color_warp, [center_pts], False, (0, 255, 255), 3)  # yellow

        # ── Unwarp back to original perspective ──────────────────────
        unwarped = cv2.warpPerspective(color_warp, self.M_inv, (self.w, self.h))
        out = cv2.addWeighted(out, 1.0, unwarped, 0.5, 0)

        # ── Draw car center line (white dashed) ─────────────────────
        car_cx = self.w // 2
        for y_pos in range(self.h // 2, self.h, 20):
            cv2.line(out, (car_cx, y_pos), (car_cx, min(y_pos + 10, self.h - 1)),
                     (200, 200, 200), 2)

        # ── Offset arrow indicator at bottom ─────────────────────────
        off = result.lateral_offset_m
        arrow_len = int(off / self.XM_PER_PIX * 0.3)  # scaled
        arrow_len = max(-120, min(120, arrow_len))
        arrow_y = self.h - 40
        cv2.arrowedLine(out, (car_cx, arrow_y),
                        (car_cx + arrow_len, arrow_y),
                        (0, 255, 255), 3, tipLength=0.3)

        # ── Text HUD ─────────────────────────────────────────────────
        off_dir = "RIGHT" if off > 0 else "LEFT"
        curv = result.curvature_m
        conf_pct = result.confidence * 100

        texts = [
            (f"Curvature : {curv:.0f} m", (0, 255, 180)),
            (f"Offset    : {abs(off):.2f} m {off_dir}", (255, 255, 100)),
            (f"Confidence: {conf_pct:.0f}%", (180, 180, 255)),
        ]
        for i, (txt, col) in enumerate(texts):
            cv2.putText(out, txt, (20, 40 + i * 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

        return out

