"""
views/lane_camera.py — Front-facing RGB camera for lane detection

Spawns a camera sensor on the vehicle hood facing forward.
Provides thread-safe frame access for the detection pipeline.
"""

import threading
import numpy as np
import carla

from config import (LANE_CAM_W, LANE_CAM_H, LANE_CAM_FOV,
                    LANE_CAM_X, LANE_CAM_Z, LANE_CAM_PITCH)
from utils.logger import log


class LaneCamera:
    """
    Front-facing RGB camera sensor attached to the vehicle.

    Usage:
        cam = LaneCamera(world, vehicle)
        # ... after world.tick() ...
        frame = cam.frame          # BGR numpy array or None
        count = cam.frame_count    # total frames received
    """

    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.sensor = None
        self._frame = None
        self._lock = threading.Lock()
        self._frame_count = 0

        self._setup()

    def _setup(self):
        """Spawn and configure the front camera sensor."""
        bp_lib = self.world.get_blueprint_library()
        bp = bp_lib.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(LANE_CAM_W))
        bp.set_attribute("image_size_y", str(LANE_CAM_H))
        bp.set_attribute("fov", str(LANE_CAM_FOV))
        bp.set_attribute("sensor_tick", "0.0")

        # Mount on vehicle hood, looking forward and slightly down
        transform = carla.Transform(
            carla.Location(x=LANE_CAM_X, z=LANE_CAM_Z),
            carla.Rotation(pitch=LANE_CAM_PITCH)
        )

        self.sensor = self.world.spawn_actor(
            bp, transform, attach_to=self.vehicle)
        self.sensor.listen(self._on_frame)
        log(f"LaneCamera {LANE_CAM_W}x{LANE_CAM_H} FOV={LANE_CAM_FOV}")

    def _on_frame(self, image):
        """Camera callback — convert BGRA to BGR numpy array."""
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        bgr = arr[:, :, :3].copy()
        with self._lock:
            self._frame = bgr
            self._frame_count += 1

    @property
    def frame(self):
        """Latest camera frame as BGR numpy array (thread-safe). None if no frame yet."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    @property
    def frame_count(self):
        """Total number of frames received from the sensor."""
        with self._lock:
            return self._frame_count

    def destroy(self):
        """Clean up the camera sensor."""
        if self.sensor:
            try:
                self.sensor.stop()
                self.sensor.destroy()
            except Exception:
                pass
            self.sensor = None
