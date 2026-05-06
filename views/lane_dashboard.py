"""
views/lane_dashboard.py — Real-time lane detection dashboard

Multi-panel Pygame window showing:
  • Main camera feed with lane overlay (large)
  • Binary threshold view (small)
  • Bird's-eye warped view (small)
  • HUD: speed, curvature, offset, confidence, steering
  • Visual steering indicator bar
"""

import math
import numpy as np

from config import HAS_PYGAME, LANE_CAM_W, LANE_CAM_H
from utils.logger import log
from models.lane_detector import LaneResult

if HAS_PYGAME:
    import pygame


# Layout constants
MAIN_W, MAIN_H = 960, 540          # main camera view
THUMB_W, THUMB_H = 240, 135        # small thumbnail views
PANEL_W = THUMB_W + 20             # right panel width
WIN_W = MAIN_W + PANEL_W
WIN_H = MAIN_H

# Colors
C_BG      = (18, 18, 24)
C_PANEL   = (25, 25, 35)
C_GREEN   = (0, 220, 90)
C_RED     = (220, 50, 50)
C_YELLOW  = (255, 210, 60)
C_CYAN    = (80, 220, 255)
C_WHITE   = (220, 220, 220)
C_GRAY    = (120, 120, 120)
C_DARK    = (40, 40, 50)


class LaneDashboard:
    """
    Pygame dashboard for lane detection visualization.

    Usage:
        dash = LaneDashboard()
        alive = dash.render(lane_result, speed_kmh, steer, frame_num)
        # alive = False if user closed window
    """

    def __init__(self):
        if not HAS_PYGAME:
            raise RuntimeError("pygame required: pip install pygame")

        pygame.init()
        pygame.display.set_caption("Autonex — Lane Detection Dashboard")
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.clock = pygame.time.Clock()

        self.font_lg = pygame.font.SysFont("Consolas", 20, bold=True)
        self.font_md = pygame.font.SysFont("Consolas", 16, bold=True)
        self.font_sm = pygame.font.SysFont("Consolas", 13)

        log(f"LaneDashboard {WIN_W}x{WIN_H}")

    def render(self, lane: LaneResult, speed_kmh: float,
               steer: float, frame: int) -> bool:
        """
        Draw one frame. Returns False if user wants to quit.
        """
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                return False

        self.screen.fill(C_BG)

        # ── Main camera view ─────────────────────────────────────────
        if lane.overlay is not None:
            self._blit_cv(lane.overlay, (0, 0), (MAIN_W, MAIN_H))

        # ── Right panel background ───────────────────────────────────
        panel_x = MAIN_W
        pygame.draw.rect(self.screen, C_PANEL,
                         (panel_x, 0, PANEL_W, WIN_H))

        # ── Thumbnail: Binary ────────────────────────────────────────
        tx = panel_x + 10
        ty = 10
        self._draw_label(tx, ty, "BINARY THRESHOLD", C_CYAN)
        ty += 20
        if lane.binary is not None:
            self._blit_gray(lane.binary, (tx, ty), (THUMB_W, THUMB_H))
        ty += THUMB_H + 10

        # ── Thumbnail: Warped ────────────────────────────────────────
        self._draw_label(tx, ty, "BIRD'S EYE VIEW", C_CYAN)
        ty += 20
        if lane.warped is not None:
            self._blit_gray(lane.warped, (tx, ty), (THUMB_W, THUMB_H))
        ty += THUMB_H + 15

        # ── Status ───────────────────────────────────────────────────
        det_col = C_GREEN if lane.detected else C_RED
        det_txt = "LANE DETECTED" if lane.detected else "LANE LOST"
        self._draw_label(tx, ty, det_txt, det_col)
        ty += 25

        # ── Telemetry ────────────────────────────────────────────────
        conf_pct = lane.confidence * 100
        off = lane.lateral_offset_m
        off_dir = "R" if off > 0 else "L"
        curv = lane.curvature_m

        entries = [
            ("Speed", f"{speed_kmh:.1f} km/h",
             C_GREEN if speed_kmh > 2 else C_RED),
            ("Curvature", f"{curv:.0f} m" if curv < 9999 else "Straight",
             C_WHITE),
            ("Offset", f"{abs(off):.2f}m {off_dir}",
             C_YELLOW if abs(off) > 0.3 else C_GREEN),
            ("Confidence", f"{conf_pct:.0f}%",
             C_GREEN if conf_pct > 50 else C_RED),
            ("Frame", f"{frame}", C_GRAY),
        ]

        for label, val, col in entries:
            self._draw_label(tx, ty, f"{label:12s}: {val}", col)
            ty += 22

        # ── Steering indicator ───────────────────────────────────────
        ty += 10
        self._draw_label(tx, ty, "STEERING", C_CYAN)
        ty += 20
        self._draw_steering_bar(tx, ty, THUMB_W, 16, steer)
        ty += 30

        # ── Offset indicator ─────────────────────────────────────────
        self._draw_label(tx, ty, "LANE POSITION", C_CYAN)
        ty += 20
        self._draw_offset_bar(tx, ty, THUMB_W, 16, off)

        # ── Controls hint ────────────────────────────────────────────
        self._draw_label(tx, WIN_H - 25, "ESC → Exit", C_GRAY)

        pygame.display.flip()
        self.clock.tick(30)
        return True

    # ─── Helper: blit OpenCV BGR image ───────────────────────────────

    def _blit_cv(self, img, pos, size):
        """Draw a BGR numpy array onto the pygame surface."""
        rgb = img[:, :, ::-1]  # BGR → RGB
        surf = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
        if (img.shape[1], img.shape[0]) != size:
            surf = pygame.transform.scale(surf, size)
        self.screen.blit(surf, pos)

    def _blit_gray(self, img, pos, size):
        """Draw a grayscale numpy array onto the pygame surface."""
        rgb = np.stack([img, img, img], axis=-1)
        surf = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
        if (img.shape[1], img.shape[0]) != size:
            surf = pygame.transform.scale(surf, size)
        self.screen.blit(surf, pos)

    # ─── Helper: text label ──────────────────────────────────────────

    def _draw_label(self, x, y, text, color):
        ts = self.font_sm.render(text, True, color)
        self.screen.blit(ts, (x, y))

    # ─── Helper: steering bar ────────────────────────────────────────

    def _draw_steering_bar(self, x, y, w, h, steer):
        """Horizontal bar: center = straight, left/right = steering."""
        pygame.draw.rect(self.screen, C_DARK, (x, y, w, h))
        center = x + w // 2
        # Indicator position
        pos = center + int(steer * (w // 2))
        pos = max(x + 4, min(x + w - 4, pos))
        # Draw center line
        pygame.draw.line(self.screen, C_GRAY, (center, y), (center, y + h), 1)
        # Draw indicator
        col = C_GREEN if abs(steer) < 0.15 else C_YELLOW if abs(steer) < 0.5 else C_RED
        pygame.draw.rect(self.screen, col, (pos - 3, y, 6, h))

    # ─── Helper: offset bar ──────────────────────────────────────────

    def _draw_offset_bar(self, x, y, w, h, offset_m):
        """Horizontal bar showing lateral offset from lane center."""
        pygame.draw.rect(self.screen, C_DARK, (x, y, w, h))
        center = x + w // 2
        # Scale: ±1.5m maps to full bar width
        px_per_m = (w // 2) / 1.5
        pos = center + int(offset_m * px_per_m)
        pos = max(x + 4, min(x + w - 4, pos))
        pygame.draw.line(self.screen, C_GRAY, (center, y), (center, y + h), 1)
        col = C_GREEN if abs(offset_m) < 0.3 else C_YELLOW if abs(offset_m) < 0.7 else C_RED
        pygame.draw.rect(self.screen, col, (pos - 3, y, 6, h))

    def destroy(self):
        """Cleanup pygame."""
        if HAS_PYGAME:
            try:
                pygame.quit()
            except Exception:
                pass
