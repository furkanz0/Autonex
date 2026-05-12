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


def pick_spawn(wmap, near, toward):
    """
    Select the closest spawn point to 'near' from the map's
    spawn_points() list that faces the 'toward' direction.
    Prefers closer spawn points more aggressively.
    """
    sec("3 – Spawn Point Selection")
    spawns = wmap.get_spawn_points()
    log(f"Map has {len(spawns)} spawn points")

    dx = toward.x - near.x
    dy = toward.y - near.y
    mag = math.sqrt(dx*dx + dy*dy) or 1
    dx /= mag; dy /= mag

    best, best_score = None, -1e9
    for sp in spawns:
        dist = sp.location.distance(near)
        if dist > 500:
            continue
        yr = math.radians(sp.rotation.yaw)
        dot = math.cos(yr)*dx + math.sin(yr)*dy
        # Heavier distance penalty so we spawn closer to start
        score = dot * 10 - dist * 0.05
        if score > best_score:
            best_score = score
            best = sp

    if best is None:
        best = spawns[0]
        print("  [!] No suitable spawn found, using first spawn")

    log(f"Selected spawn: ({best.location.x:.1f}, {best.location.y:.1f})  "
        f"yaw={best.rotation.yaw:.1f}°  score={best_score:.2f}")
    return best


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
            route = grp.trace_route(start_loc, end_loc)
            wps   = [r[0] for r in route]
            if len(wps) > 0:
                log(f"GlobalRoutePlanner: {len(wps)} waypoints")
                return wps
            else:
                print(f"  [!] GRP returned {len(wps)} waypoints, falling back")
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

    log(f"next() chain: {len(wps)} waypoints")
    return wps
