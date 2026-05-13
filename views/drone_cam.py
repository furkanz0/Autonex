"""
views/drone_cam.py — Pygame drone camera window and HUD
"""

import numpy as np
import carla

from config import HAS_PYGAME, W, H, DFOV, DH, DB, DP
from utils.logger import log

if HAS_PYGAME:
    import pygame


class DroneCam:
    """Overhead drone camera + pygame HUD window."""

    def __init__(self, world, vehicle, end_loc, start_loc=None):
        self.v    = vehicle
        self.end  = end_loc
        self.img  = None
        self.disp = None
        self.sens = None
        self.font = None

        # (progress bar removed)

        if not HAS_PYGAME:
            return

        pygame.init()
        pygame.display.set_caption("🚁 Town04 Tesla")
        self.disp = pygame.display.set_mode((W, H))
        self.font = pygame.font.SysFont("Consolas", 17, bold=True)

        lib = world.get_blueprint_library()
        bp  = lib.find("sensor.camera.rgb")
        for k, val in [("image_size_x", W), ("image_size_y", H),
                       ("fov", DFOV), ("sensor_tick", "0.0")]:
            bp.set_attribute(str(k), str(val))

        self.sens = world.spawn_actor(
            bp,
            carla.Transform(carla.Location(x=DB, z=DH),
                            carla.Rotation(pitch=DP)),
            attach_to=vehicle)
        self.sens.listen(self._cb)
        log(f"DroneCam {W}×{H}  FOV={DFOV}")

    def _cb(self, img):
        """Camera callback — convert incoming frame to numpy array."""
        a = np.frombuffer(img.raw_data, dtype=np.uint8)
        a = a.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
        self.img = a

    def render(self, speed, dist) -> bool:
        """
        Update the pygame window.
        Returns False → window closed (ESC / X).
        """
        if not HAS_PYGAME or self.disp is None:
            return True
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                return False

        if self.img is not None:
            self.disp.blit(
                pygame.surfarray.make_surface(self.img.swapaxes(0, 1)), (0, 0))

            spd_col = (255, 80, 80) if speed < 2 else (0, 230, 100)
            lines = [
                (f"Speed    : {speed:5.1f} km/h", spd_col),
                (f"To Goal  : {dist:6.0f} m",     (255, 255, 255)),
                ("ESC → Exit",                     (120, 120, 120)),
            ]
            y = 10
            for txt, col in lines:
                ts = self.font.render(txt, True, col)
                bg = pygame.Surface((ts.get_width() + 12, ts.get_height() + 4))
                bg.set_alpha(155)
                bg.fill((0, 0, 0))
                self.disp.blit(bg, (8, y - 2))
                self.disp.blit(ts, (12, y))
                y += 26

            pygame.display.flip()
        return True

    def destroy(self):
        """Clean up sensor and pygame."""
        if self.sens:
            try:
                self.sens.stop()
                self.sens.destroy()
            except Exception:
                pass
        if HAS_PYGAME and self.disp:
            try:
                pygame.quit()
            except Exception:
                pass
