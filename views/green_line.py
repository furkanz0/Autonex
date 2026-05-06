"""
views/green_line.py — Green waypoint line (point series on the road)
"""

import carla

from config import GL_N, GL_STEP, GL_LIFE, GL_SZ, GL_COL


def green_line(world, wmap, vehicle):
    """Draw green waypoint dots ahead of the vehicle."""
    try:
        wp = wmap.get_waypoint(vehicle.get_location(),
                               project_to_road=True,
                               lane_type=carla.LaneType.Driving)
        if not wp:
            return
        cur = wp
        for _ in range(GL_N):
            world.debug.draw_point(
                cur.transform.location + carla.Location(z=0.3),
                size=GL_SZ, color=GL_COL,
                life_time=GL_LIFE, persistent_lines=False)
            nxt = cur.next(GL_STEP)
            if not nxt:
                break
            cur = nxt[0]
    except Exception:
        pass
