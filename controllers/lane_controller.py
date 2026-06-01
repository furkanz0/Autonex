"""
controllers/lane_controller.py — PID controller for lane-following

Simple, responsive PID controller. Uses lateral offset from lane
detector directly with minimal smoothing to maintain real-time response.
"""

import math
import carla

from config import (
    LANE_KP, LANE_KI, LANE_KD,
    LANE_HEADING_KP, LANE_LOOKAHEAD_M, LANE_CHANGE_LOOKAHEAD_M,
    LANE_CHANGE_KMH, LANE_CHANGE_CENTER_TOL_M, LANE_CHANGE_STABLE_FRAMES,
    LANE_CHANGE_SETTLE_FRAMES, LANE_MAX_STEER_DELTA,
    LANE_CRUISE_KMH, LANE_CURVE_KMH, LANE_CURVE_RADIUS_THR,
    THROTTLE_CRUISE, BRAKE_KP,
    LANE_CHANGE_TARGET_M, LANE_CHANGE_RAMP_FRAMES,
    LANE_CHANGE_MIN_FRAMES, LANE_CHANGE_MAX_FRAMES,
    LANE_CHANGE_COMPLETE_RATIO,
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
        self._lane_change_dir = 0
        self._lane_change_frame = 0
        self._target_offset_m = 0.0
        self._map = None
        self._last_pid = {}
        self._target_road_id = None
        self._target_lane_id = None
        self._lane_change_cooldown = 0
        self._settle_road_id = None
        self._settle_lane_id = None
        self._settle_frames = 0
        self._lane_change_stable_frames = 0
        self._debug_lane_wp = None

    @property
    def lane_change_state(self) -> str:
        if self._lane_change_dir < 0:
            return "LEFT"
        if self._lane_change_dir > 0:
            return "RIGHT"
        return "CENTER"

    @property
    def target_offset_m(self) -> float:
        return self._target_offset_m

    @property
    def last_pid(self) -> dict:
        return self._last_pid.copy()

    def request_lane_change(self, direction: str) -> bool:
        """Start a PID-based lane change. direction: left/right."""
        if self._lane_change_dir != 0 or self._lane_change_cooldown > 0:
            return False

        direction = direction.lower().strip()
        if direction in ("left", "l", "-1"):
            self._lane_change_dir = -1
        elif direction in ("right", "r", "1"):
            self._lane_change_dir = 1
        else:
            return False

        self._lane_change_frame = 0
        self._target_offset_m = 0.0
        self._integral = 0.0
        self._target_road_id = None
        self._target_lane_id = None
        self._lane_change_stable_frames = 0
        return True

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

        # ── Steering PID (Pure Pursuit Inspired) ─────────────────────
        # In CARLA: positive steer = RIGHT, negative steer = LEFT
        # offset > 0 → lane center is right → car is left → need RIGHT → positive steer
        self._update_lane_change(lane)
        error = lane.lateral_offset_m + self._target_offset_m

        p = LANE_KP * error

        self._integral += error
        self._integral = max(-5.0, min(5.0, self._integral))
        i = LANE_KI * self._integral

        d = LANE_KD * (error - self._prev_error)
        self._prev_error = error

        raw_steer = p + i + d
        raw_steer = max(-self.MAX_STEER, min(self.MAX_STEER, raw_steer))

        # Responsive EMA smoothing (vision mode)
        # Smoothing (0.50) çok fazla faz gecikmesine (phase lag) sebep olup savrulma yaratıyordu.
        # Daha keskin tepki (0.85) ile savrulmayı engelliyoruz:
        steer = 0.85 * raw_steer + 0.15 * self._prev_steer
        self._prev_steer = steer

        # ── Speed Control ────────────────────────────────────────────
        target_kmh = self._target_speed(lane.curvature_m, abs(steer))
        if self._lane_change_dir != 0:
            target_kmh = min(target_kmh, LANE_CURVE_KMH)
        speed_ms = speed_kmh / 3.6
        target_ms = target_kmh / 3.6
        speed_err = target_ms - speed_ms

        if speed_err > 0:
            throttle = min(THROTTLE_CRUISE + 0.2 * (speed_err / (target_ms + 1e-6)), 0.8)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(BRAKE_KP * abs(speed_err) / (target_ms + 1e-6), 0.4)

        self._last_pid = {"steer": steer}

        return carla.VehicleControl(
            throttle=float(throttle),
            steer=float(steer),
            brake=float(brake),
            hand_brake=False,
            manual_gear_shift=False,
        )

    def compute_map(self, world, vehicle, speed_kmh: float) -> carla.VehicleControl:
        """Compute control from CARLA lane waypoints with the same PID core.

        Vision is still rendered by LaneDetector, but this control path uses
        CARLA's road graph so temporary camera misses cannot throw the car off
        the road.
        """
        if self._map is None:
            self._map = world.get_map()

        loc = vehicle.get_location()
        wp = self._map.get_waypoint(
            loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if wp is None:
            return self._lost_control()

        if self._lane_change_cooldown > 0:
            self._lane_change_cooldown -= 1

        target_lane_wp = self._lane_change_target_waypoint(wp, loc)
        settle_wp = None if target_lane_wp is not None else self._settle_waypoint(wp)
        base_wp = target_lane_wp or settle_wp or wp
        self._debug_lane_wp = base_wp

        lookahead = LANE_CHANGE_LOOKAHEAD_M if (self._lane_change_dir or self._settle_frames > 0) else LANE_LOOKAHEAD_M
        next_wps = base_wp.next(max(lookahead, 1.0))
        target_wp = next_wps[0] if next_wps else base_wp
        target_tf = target_wp.transform
        target_loc = target_tf.location

        yaw_rad = math.radians(vehicle.get_transform().rotation.yaw)
        heading = carla.Vector3D(math.cos(yaw_rad), math.sin(yaw_rad), 0.0)
        to_target = target_loc - loc
        target_dist = max(math.hypot(to_target.x, to_target.y), 1e-3)
        target_dir = carla.Vector3D(to_target.x / target_dist, to_target.y / target_dist, 0.0)

        measure_wp = base_wp
        wp_right = measure_wp.transform.get_right_vector()
        lateral = loc - measure_wp.transform.location
        lateral_error = lateral.x * wp_right.x + lateral.y * wp_right.y - self._target_offset_m

        cross = heading.x * target_dir.y - heading.y * target_dir.x
        dot = max(-1.0, min(1.0, heading.x * target_dir.x + heading.y * target_dir.y))
        heading_error = math.atan2(cross, dot)

        # CARLA uses positive steer for right turns. A positive heading_error
        # means the target is to the vehicle's left, so the steer contribution
        # must be negative. A positive lateral_error means the vehicle is right
        # of the target lane center, so it must steer left as well.
        error = -lateral_error
        p = LANE_KP * error
        self._integral += error
        self._integral = max(-3.0, min(3.0, self._integral))
        i = LANE_KI * self._integral
        d = LANE_KD * (error - self._prev_error)
        self._prev_error = error

        if self._lane_change_dir or self._settle_frames > 0:
            raw_steer = self._lane_change_steer(lateral_error, heading_error, target_dist)
        else:
            raw_steer = p + i + d - LANE_HEADING_KP * heading_error
        if self._lane_change_dir or self._settle_frames > 0:
            max_steer = 0.26
        elif abs(heading_error) > 0.25:
            max_steer = 0.28
        else:
            max_steer = self.MAX_STEER
        raw_steer = max(-max_steer, min(max_steer, raw_steer))
        steer = self._limit_steer_rate(raw_steer)

        if self._lane_change_dir or self._settle_frames > 0:
            target_kmh = min(LANE_CHANGE_KMH, LANE_CURVE_KMH)
        elif wp.is_junction or abs(heading_error) > 0.25 or abs(steer) > 0.20:
            target_kmh = LANE_CURVE_KMH
        else:
            target_kmh = LANE_CRUISE_KMH
        throttle, brake = self._speed_control(speed_kmh, target_kmh)

        self._last_pid = {
            "mode": "MAP_PID",
            "error": error,
            "lateral_error": lateral_error,
            "heading_error": heading_error,
            "p": p,
            "i": i,
            "d": d,
            "steer": steer,
            "target_offset": self._target_offset_m,
            "target_lane": self._target_lane_id,
            "settle_lane": self._settle_lane_id,
            "settle_frames": self._settle_frames,
            "stable_frames": self._lane_change_stable_frames,
            "target_dist": target_dist,
        }

        return carla.VehicleControl(
            throttle=float(throttle),
            steer=float(steer),
            brake=float(brake),
            hand_brake=False,
            manual_gear_shift=False,
        )

    def compute_route(self, vehicle, speed_kmh: float, waypoints, wp_idx: int):
        """Compute lane-center control against a pre-computed route."""
        if not waypoints or wp_idx >= len(waypoints):
            return self._lost_control(), wp_idx

        loc = vehicle.get_location()
        lookahead = max(LANE_LOOKAHEAD_M, 1.0)

        while wp_idx < len(waypoints) - 1:
            wp_loc = waypoints[wp_idx].transform.location
            if loc.distance(wp_loc) > lookahead:
                break
            wp_idx += 1

        target_wp = waypoints[wp_idx]
        self._debug_lane_wp = target_wp

        search_from = max(0, wp_idx - 8)
        search_to = min(len(waypoints), wp_idx + 8)
        measure_wp = min(
            waypoints[search_from:search_to],
            key=lambda wp: loc.distance(wp.transform.location),
        )

        target_loc = target_wp.transform.location
        to_target = target_loc - loc
        target_dist = max(math.hypot(to_target.x, to_target.y), 1e-3)
        target_dir = carla.Vector3D(to_target.x / target_dist, to_target.y / target_dist, 0.0)

        yaw_rad = math.radians(vehicle.get_transform().rotation.yaw)
        heading = carla.Vector3D(math.cos(yaw_rad), math.sin(yaw_rad), 0.0)

        wp_right = measure_wp.transform.get_right_vector()
        lateral = loc - measure_wp.transform.location
        lateral_error = lateral.x * wp_right.x + lateral.y * wp_right.y

        cross = heading.x * target_dir.y - heading.y * target_dir.x
        dot = max(-1.0, min(1.0, heading.x * target_dir.x + heading.y * target_dir.y))
        heading_error = math.atan2(cross, dot)

        error = -lateral_error
        p = LANE_KP * error
        self._integral += error
        self._integral = max(-3.0, min(3.0, self._integral))
        i = LANE_KI * self._integral
        d = LANE_KD * (error - self._prev_error)
        self._prev_error = error

        raw_steer = p + i + d - LANE_HEADING_KP * heading_error
        max_steer = 0.28 if abs(heading_error) > 0.25 else self.MAX_STEER
        raw_steer = max(-max_steer, min(max_steer, raw_steer))
        steer = self._limit_steer_rate(raw_steer)

        target_kmh = LANE_CURVE_KMH if abs(heading_error) > 0.22 or abs(steer) > 0.18 else LANE_CRUISE_KMH
        throttle, brake = self._speed_control(speed_kmh, target_kmh)

        self._last_pid = {
            "mode": "ROUTE_PID",
            "wp_idx": wp_idx,
            "error": error,
            "lateral_error": lateral_error,
            "heading_error": heading_error,
            "steer": steer,
            "target_dist": target_dist,
        }

        return carla.VehicleControl(
            throttle=float(throttle),
            steer=float(steer),
            brake=float(brake),
            hand_brake=False,
            manual_gear_shift=False,
        ), wp_idx

    def _target_speed(self, curvature_m: float, abs_steer: float) -> float:
        if abs_steer > 0.15:
            return LANE_CURVE_KMH
        if curvature_m < LANE_CURVE_RADIUS_THR:
            ratio = max(curvature_m / LANE_CURVE_RADIUS_THR, 0.3)
            return LANE_CURVE_KMH + (LANE_CRUISE_KMH - LANE_CURVE_KMH) * ratio
        return LANE_CRUISE_KMH

    def _speed_control(self, speed_kmh: float, target_kmh: float):
        speed_ms = speed_kmh / 3.6
        target_ms = target_kmh / 3.6
        speed_err = target_ms - speed_ms

        if speed_err > 0:
            throttle = min(THROTTLE_CRUISE + 0.25 * (speed_err / (target_ms + 1e-6)), 0.75)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(BRAKE_KP * abs(speed_err) / (target_ms + 1e-6), 0.45)
        return throttle, brake

    def _limit_steer_rate(self, raw_steer: float) -> float:
        delta = raw_steer - self._prev_steer
        delta = max(-LANE_MAX_STEER_DELTA, min(LANE_MAX_STEER_DELTA, delta))
        steer = self._prev_steer + delta
        steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))
        self._prev_steer = steer
        return steer

    def _lane_change_steer(self, lateral_error: float, heading_error: float, target_dist: float) -> float:
        """Stable steering law for merging onto the selected yellow lane center."""
        lateral_term = -0.18 * lateral_error
        heading_term = -0.58 * heading_error
        distance_gain = max(0.45, min(1.0, target_dist / max(LANE_CHANGE_LOOKAHEAD_M, 1.0)))
        return (lateral_term + heading_term) * distance_gain

    def _lost_control(self):
        steer = self._limit_steer_rate(0.0)
        self._last_pid = {"mode": "NO_WAYPOINT", "steer": steer}
        return carla.VehicleControl(throttle=0.0, brake=0.5, steer=float(steer))

    @property
    def debug_lane_waypoint(self):
        return self._debug_lane_wp

    def _lane_change_target_waypoint(self, current_wp, loc):
        if self._lane_change_dir == 0:
            self._target_offset_m = 0.0
            self._target_road_id = None
            self._target_lane_id = None
            return None

        self._lane_change_frame += 1
        if self._target_lane_id is None:
            adjacent = self._adjacent_lane(current_wp)
            if adjacent is None:
                self._abort_lane_change()
                return None
            self._target_road_id = adjacent.road_id
            self._target_lane_id = adjacent.lane_id
            self._target_offset_m = 0.0

        if (current_wp.road_id == self._target_road_id and
                current_wp.lane_id == self._target_lane_id):
            target_wp = current_wp
        else:
            target_wp = self._waypoint_on_target_lane(current_wp)
        if target_wp is None:
            target_wp = self._adjacent_lane(current_wp)
        if target_wp is None:
            self._abort_lane_change()
            return None

        right = target_wp.transform.get_right_vector()
        lateral = loc - target_wp.transform.location
        lateral_m = lateral.x * right.x + lateral.y * right.y
        center_dist = math.hypot(
            loc.x - target_wp.transform.location.x,
            loc.y - target_wp.transform.location.y,
        )
        reached_lane = current_wp.road_id == self._target_road_id and current_wp.lane_id == self._target_lane_id
        reached_center = abs(lateral_m) < LANE_CHANGE_CENTER_TOL_M or center_dist < LANE_CHANGE_CENTER_TOL_M
        timeout = self._lane_change_frame >= LANE_CHANGE_MAX_FRAMES

        if reached_lane and reached_center:
            self._lane_change_stable_frames += 1
        else:
            self._lane_change_stable_frames = 0

        if self._lane_change_stable_frames >= LANE_CHANGE_STABLE_FRAMES:
            self._finish_lane_change()
            return None

        if timeout:
            self._finish_lane_change()
            return None

        return target_wp

    def _settle_waypoint(self, current_wp):
        if self._settle_frames <= 0:
            self._settle_road_id = None
            self._settle_lane_id = None
            return None

        self._settle_frames -= 1
        if (current_wp.road_id == self._settle_road_id and
                current_wp.lane_id == self._settle_lane_id):
            return current_wp

        try:
            return self._map.get_waypoint_xodr(
                self._settle_road_id,
                self._settle_lane_id,
                current_wp.s,
            )
        except Exception:
            self._settle_frames = 0
            self._settle_road_id = None
            self._settle_lane_id = None
            return None

    def _adjacent_lane(self, wp):
        desired_side = 1 if self._lane_change_dir > 0 else -1
        right = wp.transform.get_right_vector()
        forward = wp.transform.get_forward_vector()
        candidates = []

        for adjacent in (wp.get_left_lane(), wp.get_right_lane()):
            if adjacent is None:
                continue
            same_direction = (
                wp.lane_id == 0 or adjacent.lane_id == 0 or
                (wp.lane_id > 0) == (adjacent.lane_id > 0)
            )
            if adjacent.lane_type != carla.LaneType.Driving or not same_direction:
                continue

            adj_forward = adjacent.transform.get_forward_vector()
            same_heading = (forward.x * adj_forward.x + forward.y * adj_forward.y) > 0.65
            if not same_heading or not self._lane_marking_allows(wp, desired_side):
                continue

            delta = adjacent.transform.location - wp.transform.location
            side = delta.x * right.x + delta.y * right.y
            if side * desired_side > 0.2:
                candidates.append((abs(side), adjacent))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    @staticmethod
    def _lane_marking_allows(wp, desired_side: int) -> bool:
        marking = wp.right_lane_marking if desired_side > 0 else wp.left_lane_marking
        lane_change = getattr(marking, "lane_change", None)
        if lane_change is None:
            return True

        text = str(lane_change).lower()
        if "both" in text:
            return True
        if desired_side > 0:
            return "right" in text
        return "left" in text

    def _waypoint_on_target_lane(self, current_wp):
        if self._target_road_id is None or self._target_lane_id is None:
            return None
        try:
            return self._map.get_waypoint_xodr(
                self._target_road_id,
                self._target_lane_id,
                current_wp.s,
            )
        except Exception:
            return None

    def _finish_lane_change(self):
        finished_road_id = self._target_road_id
        finished_lane_id = self._target_lane_id
        self._lane_change_dir = 0
        self._lane_change_frame = 0
        self._target_offset_m = 0.0
        self._target_road_id = None
        self._target_lane_id = None
        self._lane_change_cooldown = 70
        self._settle_road_id = finished_road_id
        self._settle_lane_id = finished_lane_id
        self._settle_frames = LANE_CHANGE_SETTLE_FRAMES if finished_lane_id is not None else 0
        self._lane_change_stable_frames = 0
        self._integral = 0.0
        self._prev_error = 0.0

    def _abort_lane_change(self):
        self._lane_change_dir = 0
        self._lane_change_frame = 0
        self._target_offset_m = 0.0
        self._target_road_id = None
        self._target_lane_id = None
        self._lane_change_cooldown = 20
        self._settle_road_id = None
        self._settle_lane_id = None
        self._settle_frames = 0
        self._lane_change_stable_frames = 0
        self._integral = 0.0
        self._prev_error = 0.0

    def _update_lane_change(self, lane: LaneResult):
        if self._lane_change_dir == 0:
            self._target_offset_m = 0.0
            return

        self._lane_change_frame += 1
        ramp = min(self._lane_change_frame / max(LANE_CHANGE_RAMP_FRAMES, 1), 1.0)
        self._target_offset_m = self._lane_change_dir * LANE_CHANGE_TARGET_M * ramp

        complete_offset = LANE_CHANGE_TARGET_M * LANE_CHANGE_COMPLETE_RATIO
        if self._lane_change_dir > 0:
            crossed_old_center = lane.lateral_offset_m <= -complete_offset
        else:
            crossed_old_center = lane.lateral_offset_m >= complete_offset
        timeout = self._lane_change_frame >= LANE_CHANGE_MAX_FRAMES

        if self._lane_change_frame >= LANE_CHANGE_MIN_FRAMES and (crossed_old_center or timeout):
            self._lane_change_dir = 0
            self._lane_change_frame = 0
            self._target_offset_m = 0.0
            self._integral = 0.0
            self._prev_error = lane.lateral_offset_m

    def _update_lane_change_from_waypoint(self, wp, loc):
        if self._lane_change_dir == 0:
            self._target_offset_m = 0.0
            return

        self._lane_change_frame += 1
        lane_width = max(getattr(wp, "lane_width", LANE_CHANGE_TARGET_M), 2.8)
        ramp = min(self._lane_change_frame / max(LANE_CHANGE_RAMP_FRAMES, 1), 1.0)
        self._target_offset_m = self._lane_change_dir * lane_width * ramp

        right = wp.transform.get_right_vector()
        lateral = loc - wp.transform.location
        lateral_m = lateral.x * right.x + lateral.y * right.y
        reached = abs(lateral_m) >= lane_width * LANE_CHANGE_COMPLETE_RATIO
        timeout = self._lane_change_frame >= LANE_CHANGE_MAX_FRAMES

        if self._lane_change_frame >= LANE_CHANGE_MIN_FRAMES and (reached or timeout):
            self._lane_change_dir = 0
            self._lane_change_frame = 0
            self._target_offset_m = 0.0
            self._integral = 0.0
            self._prev_error = 0.0

    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_steer = 0.0
        self._lost_count = 0
        self._lane_change_dir = 0
        self._lane_change_frame = 0
        self._target_offset_m = 0.0
        self._last_pid = {}
        self._target_road_id = None
        self._target_lane_id = None
        self._lane_change_cooldown = 0
        self._settle_road_id = None
        self._settle_lane_id = None
        self._settle_frames = 0
        self._lane_change_stable_frames = 0
        self._debug_lane_wp = None
