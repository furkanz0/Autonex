"""
views/chase_cam.py — OpenCV-based third-person chase camera
═══════════════════════════════════════════════════════════
Replaces the CARLA spectator viewport with a programmatically
positioned OpenCV window for side-by-side multi-window layout.
"""

import numpy as np
import cv2
import math
import carla

from config import (
    CHASE_W, CHASE_H, CHASE_X, CHASE_Y,
    CHASE_FOV, CHASE_BACK, CHASE_UP, CHASE_PIT,
)
from utils.logger import log


class ChaseCam:
    """Third-person chase camera rendered via OpenCV."""

    WIN_NAME = "Chase Camera"

    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.img = None
        self.sensor = None

        # Spawn camera sensor attached to vehicle
        lib = world.get_blueprint_library()
        bp = lib.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(CHASE_W))
        bp.set_attribute("image_size_y", str(CHASE_H))
        bp.set_attribute("fov", str(CHASE_FOV))
        bp.set_attribute("sensor_tick", "0.0")

        transform = carla.Transform(
            carla.Location(x=CHASE_BACK, z=CHASE_UP),
            carla.Rotation(pitch=CHASE_PIT),
        )

        self.sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
        self.sensor.listen(self._callback)

        # Create and position the OpenCV window
        cv2.namedWindow(self.WIN_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow(self.WIN_NAME, CHASE_X, CHASE_Y)

        log(f"ChaseCam {CHASE_W}×{CHASE_H} FOV={CHASE_FOV} → OpenCV")

    def _callback(self, image):
        """Camera sensor callback — convert CARLA image to numpy BGR array."""
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))  # BGRA
        self.img = arr[:, :, :3]  # drop alpha → BGR (OpenCV native)

    def render(self, speed, dist):
        """
        Display the latest frame with HUD overlay.
        Returns False if user closed the window.
        """
        if self.img is None:
            return True

        canvas = self.img.copy()

        # HUD overlay
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 0), (320, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, canvas, 0.5, 0, canvas)

        spd_color = (0, 230, 100) if speed >= 2 else (80, 80, 255)
        cv2.putText(canvas, f"Speed : {speed:5.1f} km/h",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, spd_color, 2, cv2.LINE_AA)
        cv2.putText(canvas, f"To Goal: {dist:6.0f} m",
                    (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "CHASE CAM",
                    (12, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1, cv2.LINE_AA)

        cv2.imshow(self.WIN_NAME, canvas)
        key = cv2.waitKey(1)

        # Check if window was closed
        if key == 27:  # ESC
            return False
        try:
            if cv2.getWindowProperty(self.WIN_NAME, cv2.WND_PROP_VISIBLE) < 1:
                return False
        except Exception:
            pass

        return True

    def destroy(self):
        """Clean up sensor and OpenCV window."""
        if self.sensor:
            try:
                self.sensor.stop()
                self.sensor.destroy()
            except Exception:
                pass
        try:
            cv2.destroyWindow(self.WIN_NAME)
        except Exception:
            pass
