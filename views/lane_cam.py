"""
views/lane_cam.py — Front-facing camera for lane detection

Aracın kaputuna monte edilmiş RGB kamera sensörü.
Her frame'i numpy array olarak saklar, LaneController tarafından kullanılır.

Teknik detaylar:
  • Çözünürlük : 640×480
  • FOV        : 110° (geniş açı, şerit kenarlarını yakalamak için)
  • Konum      : x=2.2m (aracın önü), z=1.4m (kaput üstü)
  • Pitch      : -8° (hafif aşağı, yol yüzeyine bakış)
"""

import numpy as np
import carla

from config import (
    LANE_CAM_W, LANE_CAM_H, LANE_CAM_FOV,
    LANE_CAM_X, LANE_CAM_Z, LANE_CAM_PIT,
)
from utils.logger import log


class LaneCam:
    """
    CARLA'ya bağlı ön kamera sensörü.

    Kullanım:
        cam = LaneCam(world, vehicle)
        ...
        frame = cam.get_frame()   # np.ndarray (H, W, 3) BGR veya None
        ...
        cam.destroy()
    """

    def __init__(self, world, vehicle):
        self._frame = None
        self._sensor = None

        lib = world.get_blueprint_library()
        bp = lib.find("sensor.camera.rgb")

        # Kamera ayarları
        bp.set_attribute("image_size_x", str(LANE_CAM_W))
        bp.set_attribute("image_size_y", str(LANE_CAM_H))
        bp.set_attribute("fov", str(LANE_CAM_FOV))
        bp.set_attribute("sensor_tick", "0.0")  # Her tick'te çek

        # Sensör konumu: aracın önünde, kaput üstünde, hafif aşağı bakıyor
        transform = carla.Transform(
            carla.Location(x=LANE_CAM_X, z=LANE_CAM_Z),
            carla.Rotation(pitch=LANE_CAM_PIT),
        )

        self._sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
        self._sensor.listen(self._on_image)

        log(f"LaneCam {LANE_CAM_W}×{LANE_CAM_H}  FOV={LANE_CAM_FOV}  "
            f"pos=({LANE_CAM_X}, {LANE_CAM_Z})  pitch={LANE_CAM_PIT}°")

    # ─── Callback ─────────────────────────────────────────────────────
    def _on_image(self, image):
        """CARLA kamera callback — BGRA → BGR numpy array."""
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        # BGRA → BGR (OpenCV formatı)
        self._frame = arr[:, :, :3].copy()

    # ─── Public API ───────────────────────────────────────────────────
    def get_frame(self):
        """Son kamera frame'ini döndür (np.ndarray BGR veya None)."""
        return self._frame

    def destroy(self):
        """Sensörü durdur ve yok et."""
        if self._sensor is not None:
            try:
                self._sensor.stop()
                self._sensor.destroy()
                log("LaneCam destroyed.")
            except Exception:
                pass
            self._sensor = None
