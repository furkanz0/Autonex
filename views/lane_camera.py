"""
views/lane_camera.py — CARLA ön kamera sensörü (Lane Detection için)

Aracın kaputuna monte edilmiş RGB kamera.
Her tick'te frame'i yakalar ve .frame property'si ile erişim sağlar.

Teknik:
  • Çözünürlük : LANE_CAM_W × LANE_CAM_H (varsayılan 640×480)
  • FOV        : LANE_CAM_FOV (varsayılan 110°)
  • Konum      : Aracın önünde, kaput üstünde
  • Pitch      : Hafif aşağı (yol yüzeyine bakış)
"""

import numpy as np
import carla
import queue

from config import (
    LANE_CAM_W, LANE_CAM_H, LANE_CAM_FOV,
    LANE_CAM_X, LANE_CAM_Z, LANE_CAM_PIT,
)
from utils.logger import log


class LaneCamera:
    """
    CARLA RGB kamera sensörü.

    Kullanım:
        cam = LaneCamera(world, vehicle)
        ...
        img = cam.frame          # np.ndarray (H, W, 3) BGR veya None
        ...
        cam.destroy()
    """

    def __init__(self, world, vehicle):
        self._frame = None
        self._frame_no = None
        self._queue = queue.Queue(maxsize=2)
        self._sensor = None

        lib = world.get_blueprint_library()
        bp = lib.find("sensor.camera.rgb")

        # Kamera ayarları
        bp.set_attribute("image_size_x", str(LANE_CAM_W))
        bp.set_attribute("image_size_y", str(LANE_CAM_H))
        bp.set_attribute("fov", str(LANE_CAM_FOV))
        bp.set_attribute("sensor_tick", "0.0")  # her simülasyon tick'inde çek

        # Sensör konumu: aracın önünde, kaput üstünde, hafif aşağı bakıyor
        transform = carla.Transform(
            carla.Location(x=LANE_CAM_X, z=LANE_CAM_Z),
            carla.Rotation(pitch=LANE_CAM_PIT),
        )

        self._sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
        self._sensor.listen(self._on_image)

        log(f"LaneCamera {LANE_CAM_W}×{LANE_CAM_H}  FOV={LANE_CAM_FOV}  "
            f"pos=({LANE_CAM_X}, {LANE_CAM_Z})  pitch={LANE_CAM_PIT}°")

    # ─── Callback ─────────────────────────────────────────────────────
    def _on_image(self, image):
        """CARLA kamera callback: BGRA → BGR numpy array."""
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        frame = arr[:, :, :3].copy()  # BGRA → BGR
        self._frame = frame
        self._frame_no = image.frame
        try:
            self._queue.put_nowait((image.frame, frame))
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait((image.frame, frame))
            except queue.Full:
                pass

    # ─── Property ─────────────────────────────────────────────────────
    @property
    def frame(self):
        """Son kamera frame'i (np.ndarray BGR veya None)."""
        return self._frame

    @property
    def frame_no(self):
        """Son kamera frame numarası."""
        return self._frame_no

    def wait_for_frame(self, min_frame=None, timeout=0.2):
        """Belirli tick'e ait veya daha yeni kamera frame'ini kısa süre bekle."""
        while True:
            if min_frame is None and self._frame is not None:
                return self._frame
            if self._frame is not None and self._frame_no is not None and self._frame_no >= min_frame:
                return self._frame
            try:
                frame_no, frame = self._queue.get(timeout=timeout)
            except queue.Empty:
                return self._frame
            self._frame_no = frame_no
            self._frame = frame

    # ─── Cleanup ──────────────────────────────────────────────────────
    def destroy(self):
        """Sensörü durdur ve yok et."""
        if self._sensor is not None:
            try:
                self._sensor.stop()
                self._sensor.destroy()
                log("LaneCamera destroyed.")
            except Exception:
                pass
            self._sensor = None
