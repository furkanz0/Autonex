"""
controllers/vehicle_controller.py — PID proportional waypoint follower
"""

import math
import carla

from config import (
    KP_STEER, LOOKAHEAD_M, THROTTLE_CRUISE,
    BRAKE_KP, TARGET_SPEED_MS,
)


def compute_control(vehicle, waypoints, wp_idx):
    """
    Apply proportional steering toward the target waypoint
    LOOKAHEAD_M ahead of the vehicle.
    Throttle is determined by a simple speed controller.
    Returns: (VehicleControl, new wp_idx)
    """
    if wp_idx >= len(waypoints):
        # Route finished, stop
        ctrl = carla.VehicleControl(throttle=0.0, brake=1.0)
        return ctrl, wp_idx

    # Current transform
    t   = vehicle.get_transform()
    loc = t.location
    yaw = math.radians(t.rotation.yaw)
    fwd = (math.cos(yaw), math.sin(yaw))

    # Advance past lookahead distance – find target waypoint
    while wp_idx < len(waypoints) - 1:
        wp_loc = waypoints[wp_idx].transform.location
        if loc.distance(wp_loc) > LOOKAHEAD_M:
            break
        wp_idx += 1

    target = waypoints[wp_idx].transform.location

    # Vector from vehicle to target
    dx = target.x - loc.x
    dy = target.y - loc.y
    dist = math.sqrt(dx*dx + dy*dy) or 1.0

    # Cross product (z component): angle direction between vehicle and target
    cross = fwd[0]*dy - fwd[1]*dx
    steer = KP_STEER * cross / dist
    steer = max(-1.0, min(1.0, steer))

    # Speed control
    vel = vehicle.get_velocity()
    spd = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)   # m/s

    err = TARGET_SPEED_MS - spd
    if err > 0:
        throttle = min(THROTTLE_CRUISE + 0.3*(err/TARGET_SPEED_MS), 1.0)
        brake    = 0.0
    else:
        throttle = 0.0
        brake    = min(BRAKE_KP * abs(err) / TARGET_SPEED_MS, 0.5)

    ctrl = carla.VehicleControl(
        throttle=float(throttle),
        steer=float(steer),
        brake=float(brake),
        hand_brake=False,
        manual_gear_shift=False,
    )
    return ctrl, wp_idx
