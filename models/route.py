"""
models/route.py — Spawn point selection, target snapping and route computation
"""

import math
import carla

from config import HAS_GRP, WANT_START, WANT_END
from utils.logger import log, sec

if HAS_GRP:
    # pyrefly: ignore [missing-import]
    from agents.navigation.global_route_planner import GlobalRoutePlanner


ROUTE_TARGET_TRIM_M = 6.0
ROUTE_MIN_TAIL_TO_TRIM = 8
ROUTE_LOOP_CLOSE_M = 8.0
ROUTE_LOOP_MIN_SPAN = 24
ROUTE_CANDIDATE_RADIUS_M = 16.0
ROUTE_MAX_CANDIDATES = 10


def pick_spawn(wmap, near, toward):
    """
    Returns a safe spawn transform exactly at the 'near' location,
    snapped to the correct lane facing 'toward'.
    This prevents the vehicle from teleporting blocks away to a sparse predefined spawn point.
    """
    sec("3 – Spawn Point Selection")
    
    snapped_loc = snap_directed(wmap, near, toward)
    wp = wmap.get_waypoint(snapped_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if wp:
        spawn_tf = wp.transform
        spawn_tf.location.z += 0.5  # Lift slightly to prevent ground collision
        log(f"Spawn exactly at clicked location: ({spawn_tf.location.x:.1f}, {spawn_tf.location.y:.1f})")
        return spawn_tf
        
    # Fallback if map fails
    print("  [!] Could not snap to lane, using first available spawn point.")
    return wmap.get_spawn_points()[0]


def snap_end(wmap, loc):
    """Snap the target point to the nearest driving lane."""
    wp = wmap.get_waypoint(loc, project_to_road=True,
                           lane_type=carla.LaneType.Driving)
    if wp:
        log(f"Target snap: ({wp.transform.location.x:.1f}, {wp.transform.location.y:.1f})")
        return wp.transform.location
    log("Target snap failed, using original coordinates", "!")
    return loc

def snap_directed(wmap, loc, target_loc):
    """Snaps to the nearest lane that faces toward the target_loc (avoids opposite lanes)."""
    wp = wmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if not wp:
        return loc
        
    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y
    mag = math.sqrt(dx*dx + dy*dy) or 1
    dx, dy = dx/mag, dy/mag
    
    yaw = math.radians(wp.transform.rotation.yaw)
    wp_dx, wp_dy = math.cos(yaw), math.sin(yaw)
    
    # If the lane faces opposite to the general direction of the target
    if dx * wp_dx + dy * wp_dy < 0:
        # Check opposite direction lanes via API
        candidates = []
        left = wp.get_left_lane()
        right = wp.get_right_lane()
        if left and left.lane_type == carla.LaneType.Driving: candidates.append(left)
        if right and right.lane_type == carla.LaneType.Driving: candidates.append(right)
        
        # Check opposite direction lanes that are physically separated (different road_id)
        # by projecting points laterally (left and right up to 20 meters)
        lat_dx, lat_dy = -wp_dy, wp_dx # Leftward vector
        for dist in (5, 10, 15, 20):
            # Left side
            loc_l = carla.Location(x=loc.x + lat_dx*dist, y=loc.y + lat_dy*dist, z=loc.z)
            wp_l = wmap.get_waypoint(loc_l, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp_l and wp_l.road_id != wp.road_id: candidates.append(wp_l)
            
            # Right side
            loc_r = carla.Location(x=loc.x - lat_dx*dist, y=loc.y - lat_dy*dist, z=loc.z)
            wp_r = wmap.get_waypoint(loc_r, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp_r and wp_r.road_id != wp.road_id: candidates.append(wp_r)
        
        for cand in candidates:
            c_yaw = math.radians(cand.transform.rotation.yaw)
            c_dx, c_dy = math.cos(c_yaw), math.sin(c_yaw)
            if dx * c_dx + dy * c_dy > 0:
                wp = cand
                break
                
    return wp.transform.location


def build_route(wmap, start_loc, end_loc, world):
    """
    Use GlobalRoutePlanner if available, otherwise fall back to .next() chain.
    Returns: list of carla.Waypoint
    """
    sec("5 – Route Computation")
    if HAS_GRP:
        try:
            grp   = GlobalRoutePlanner(wmap, sampling_resolution=2.0)
            candidates = _ranked_route_candidates(wmap, grp, start_loc, end_loc)
            if candidates:
                best = candidates[0]
                log(
                    f"GlobalRoutePlanner best lane route: {len(best['wps'])} waypoints "
                    f"(score {best['score']:.1f}, length {best['length']:.1f}m)"
                )
                return best["wps"]
            print("  [!] GRP returned no valid candidate routes, falling back")
        except Exception as e:
            print(f"  [!] GRP error: {e}, falling back to .next() chain")

    # Fallback: .next() chain — try to head toward end_loc
    wp = wmap.get_waypoint(start_loc, project_to_road=True,
                           lane_type=carla.LaneType.Driving)
    wps = [wp]
    prev_dist = wp.transform.location.distance(end_loc)

    for _ in range(5000):
        nxt_list = wps[-1].next(2.0)
        if not nxt_list:
            break

        # If multiple branches, pick the one closest to end_loc
        best_wp = None
        best_dist = 1e9
        for candidate in nxt_list:
            d = candidate.transform.location.distance(end_loc)
            if d < best_dist:
                best_dist = d
                best_wp = candidate

        wps.append(best_wp)

        if best_dist < 10:
            break

        # If we've been going away for too long, stop
        if best_dist > prev_dist + 200:
            log(f"Route diverging from target, stopping at {len(wps)} wps", "!")
            break

        prev_dist = min(prev_dist, best_dist)

    wps = _postprocess_route(wps, end_loc)
    log(f"next() chain: {len(wps)} waypoints")
    return wps


def _ranked_route_candidates(wmap, grp, start_loc, end_loc):
    start_candidates = _nearby_driving_waypoints(wmap, start_loc, toward=end_loc)
    end_candidates = _nearby_driving_waypoints(wmap, end_loc)
    ranked = []

    for start_wp in start_candidates:
        for end_wp in end_candidates:
            route = grp.trace_route(
                start_wp.transform.location,
                end_wp.transform.location,
            )
            wps = [r[0] for r in route]
            if len(wps) < 2:
                continue

            wps = _postprocess_route(wps, end_wp.transform.location)
            if len(wps) < 2:
                continue

            length = _route_length(wps)
            start_dist = start_loc.distance(wps[0].transform.location)
            end_dist = end_loc.distance(wps[-1].transform.location)
            score = length + start_dist * 4.0 + end_dist * 4.0
            ranked.append({
                "wps": wps,
                "score": score,
                "length": length,
                "start_dist": start_dist,
                "end_dist": end_dist,
            })

    ranked.sort(key=lambda item: item["score"])
    if ranked:
        best = ranked[0]
        log(
            f"Route lane candidates: {len(ranked)} valid, "
            f"start offset {best['start_dist']:.1f}m, "
            f"end offset {best['end_dist']:.1f}m",
            "~",
        )
    return ranked


def _nearby_driving_waypoints(wmap, loc, toward=None):
    base = wmap.get_waypoint(
        loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if not base:
        return []

    candidates = []
    seen = set()

    def add(wp):
        if not wp or wp.lane_type != carla.LaneType.Driving:
            return
        if wp.transform.location.distance(loc) > ROUTE_CANDIDATE_RADIUS_M:
            return
        key = (wp.road_id, wp.section_id, wp.lane_id, round(wp.s, 1))
        if key in seen:
            return
        seen.add(key)
        candidates.append(wp)

    add(base)
    add(base.get_left_lane())
    add(base.get_right_lane())

    yaw = math.radians(base.transform.rotation.yaw)
    lat_dx, lat_dy = -math.sin(yaw), math.cos(yaw)
    for dist in (3.5, 7.0, 10.5, 14.0):
        for sign in (-1.0, 1.0):
            probe = carla.Location(
                x=loc.x + lat_dx * dist * sign,
                y=loc.y + lat_dy * dist * sign,
                z=loc.z,
            )
            add(wmap.get_waypoint(
                probe, project_to_road=True,
                lane_type=carla.LaneType.Driving))

    if toward:
        tx = toward.x - loc.x
        ty = toward.y - loc.y
        mag = math.sqrt(tx * tx + ty * ty) or 1.0
        tx, ty = tx / mag, ty / mag

        def direction_penalty(wp):
            yaw_rad = math.radians(wp.transform.rotation.yaw)
            dot = math.cos(yaw_rad) * tx + math.sin(yaw_rad) * ty
            return 0.0 if dot >= 0.0 else 25.0
    else:
        def direction_penalty(wp):
            return 0.0

    candidates.sort(key=lambda wp: (
        direction_penalty(wp) + wp.transform.location.distance(loc),
        abs(wp.lane_id),
    ))
    return candidates[:ROUTE_MAX_CANDIDATES]


def _route_length(wps):
    total = 0.0
    for prev, cur in zip(wps, wps[1:]):
        total += prev.transform.location.distance(cur.transform.location)
    return total


def _postprocess_route(wps, end_loc):
    wps = _remove_route_loops(wps)
    wps = _trim_route_near_target(wps, end_loc)
    return wps


def _trim_route_near_target(wps, end_loc):
    """
    Cut route tails that loop around after already passing the selected target.
    This keeps map-selected routes from drawing large block loops near the end.
    """
    if not wps or len(wps) < 3:
        return wps

    best_idx = None
    best_dist = float("inf")
    for idx, wp in enumerate(wps):
        try:
            dist = wp.transform.location.distance(end_loc)
        except Exception:
            continue
        if dist < best_dist:
            best_dist = dist
            best_idx = idx

    if best_idx is None:
        return wps

    tail_len = len(wps) - best_idx - 1
    if best_dist <= ROUTE_TARGET_TRIM_M and tail_len >= ROUTE_MIN_TAIL_TO_TRIM:
        trimmed = wps[:best_idx + 1]
        log(
            f"Route tail trimmed near target: {len(wps)} -> {len(trimmed)} "
            f"(closest {best_dist:.1f}m)",
            "~",
        )
        return trimmed

    return wps


def _remove_route_loops(wps):
    """Remove obvious geometric loops where the route returns near an old point."""
    if not wps or len(wps) < ROUTE_LOOP_MIN_SPAN:
        return wps

    cleaned = list(wps)
    changed = True
    while changed:
        changed = False
        for i in range(0, len(cleaned) - ROUTE_LOOP_MIN_SPAN):
            loc_i = cleaned[i].transform.location
            for j in range(len(cleaned) - 1, i + ROUTE_LOOP_MIN_SPAN, -1):
                loc_j = cleaned[j].transform.location
                if loc_i.distance(loc_j) > ROUTE_LOOP_CLOSE_M:
                    continue

                # Keep the later waypoint so heading/road context remains current.
                before = len(cleaned)
                cleaned = cleaned[:i + 1] + cleaned[j:]
                log(f"Route loop removed: {before} -> {len(cleaned)} waypoints", "~")
                changed = True
                break
            if changed:
                break

    return cleaned
