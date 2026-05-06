"""
models/route.py — Spawn point selection, target snapping and route computation
"""

import math
import carla

from config import HAS_GRP, WANT_START, WANT_END
from utils.logger import log, sec

if HAS_GRP:
    from agents.navigation.global_route_planner import GlobalRoutePlanner


def pick_spawn(wmap, near, toward):
    """
    Select the closest spawn point to 'near' from the map's
    spawn_points() list that faces the 'toward' direction.
    Guaranteed to be on-road and collision-free.
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
        if dist > 300:          # skip if too far
            continue
        yr = math.radians(sp.rotation.yaw)
        dot = math.cos(yr)*dx + math.sin(yr)*dy
        score = dot * 10 - dist * 0.01
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
            log(f"GlobalRoutePlanner: {len(wps)} waypoints")
            return wps
        except Exception as e:
            print(f"  [!] GRP error: {e}, falling back to .next() chain")

    # Fallback: .next() chain
    wp = wmap.get_waypoint(start_loc, project_to_road=True,
                           lane_type=carla.LaneType.Driving)
    wps = [wp]
    for _ in range(3000):
        nxt = wps[-1].next(2.0)
        if not nxt:
            break
        wps.append(nxt[0])
        if wps[-1].transform.location.distance(end_loc) < 10:
            break
    log(f"next() chain: {len(wps)} waypoints")
    return wps
