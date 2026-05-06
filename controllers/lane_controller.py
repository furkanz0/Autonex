"""
controllers/lane_controller.py — PID controller for lane-following

Simple, responsive PID controller. Uses lateral offset from lane
detector directly with minimal smoothing to maintain real-time response.
"""

import math
import carla

from config import (
    LANE_KP, LANE_KI, LANE_KD,
    LANE_CRUISE_KMH, LANE_CURVE_KMH, LANE_CURVE_RADIUS_THR,
    THROTTLE_CRUISE, BRAKE_KP,
)
from models.lane_detector import LaneResult


class LaneController:
    """
    PID-based vehicle controller using lane detection output.
    """

    MAX_STEER = 0.35

    def __init__(self):
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_steer = 0.0
        self._lost_count = 0
        self._max_lost = 120

    def compute(self, lane: LaneResult, speed_kmh: float) -> carla.VehicleControl:
        """Compute vehicle control from lane detection result."""

        if not lane.detected:
            self._lost_count += 1
            if self._lost_count > self._max_lost:
                return carla.VehicleControl(
                    throttle=0.0, brake=0.6,
                    steer=float(self._prev_steer * 0.9))
            else:
                steer = self._prev_steer * 0.97
                self._prev_steer = steer
                return carla.VehicleControl(
                    throttle=0.45, brake=0.0,
                    steer=float(steer))

        self._lost_count = 0

        # ── Steering PID (direct, minimal lag) ───────────────────────
        # In CARLA: positive steer = LEFT, negative steer = RIGHT
        # offset > 0 → lane center is right → car is left → need RIGHT → negative steer
        error = -lane.lateral_offset_m

        p = LANE_KP * error

        self._integral += error
        self._integral = max(-5.0, min(5.0, self._integral))
        i = LANE_KI * self._integral

        d = LANE_KD * (error - self._prev_error)
        self._prev_error = error

        raw_steer = p + i + d
        raw_steer = max(-self.MAX_STEER, min(self.MAX_STEER, raw_steer))

        # Light EMA smoothing (single layer only)
        steer = 0.4 * raw_steer + 0.6 * self._prev_steer
        self._prev_steer = steer

        # ── Speed Control ────────────────────────────────────────────
        target_kmh = self._target_speed(lane.curvature_m, abs(steer))
        speed_ms = speed_kmh / 3.6
        target_ms = target_kmh / 3.6
        speed_err = target_ms - speed_ms

        if speed_err > 0:
            throttle = min(THROTTLE_CRUISE + 0.2 * (speed_err / (target_ms + 1e-6)), 0.8)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(BRAKE_KP * abs(speed_err) / (target_ms + 1e-6), 0.4)

        return carla.VehicleControl(
            throttle=float(throttle),
            steer=float(steer),
            brake=float(brake),
            hand_brake=False,
            manual_gear_shift=False,
        )

    def _target_speed(self, curvature_m: float, abs_steer: float) -> float:
        if abs_steer > 0.15:
            return LANE_CURVE_KMH
        if curvature_m < LANE_CURVE_RADIUS_THR:
            ratio = max(curvature_m / LANE_CURVE_RADIUS_THR, 0.3)
            return LANE_CURVE_KMH + (LANE_CRUISE_KMH - LANE_CURVE_KMH) * ratio
        return LANE_CRUISE_KMH

    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_steer = 0.0
        self._lost_count = 0
