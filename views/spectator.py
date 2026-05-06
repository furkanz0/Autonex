"""
views/spectator.py — CARLA spectator (third-person) camera tracking
"""

import math
import carla

from config import CAM_BACK, CAM_UP, CAM_PIT


def spectator_update(spec, vehicle):
    """Position the spectator camera behind the vehicle."""
    try:
        if not vehicle.is_alive:
            return False
        t   = vehicle.get_transform()
        yaw = math.radians(t.rotation.yaw)
        spec.set_transform(carla.Transform(
            carla.Location(x=t.location.x + CAM_BACK * math.cos(yaw),
                           y=t.location.y + CAM_BACK * math.sin(yaw),
                           z=t.location.z + CAM_UP),
            carla.Rotation(pitch=CAM_PIT, yaw=t.rotation.yaw)))
        return True
    except RuntimeError:
        return False
