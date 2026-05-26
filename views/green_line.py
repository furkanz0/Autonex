"""
views/green_line.py — Green waypoint line (point series on the road)
"""

import carla

from config import GL_N, GL_STEP, GL_LIFE, GL_SZ, GL_COL


def green_line(world, vehicle, waypoints, wp_idx):
    """Draw waypoint dots ahead of the vehicle along the planned route."""
    try:
        if not waypoints:
            return
            
        # wp_idx is the lookahead target (~8m ahead).
        # We step back 4 waypoints (~8m) to start drawing from the vehicle's current location.
        start_idx = max(0, wp_idx - 4)
        end_idx = min(start_idx + GL_N, len(waypoints))
        
        for i in range(start_idx, end_idx):
            wp = waypoints[i]
            world.debug.draw_point(
                wp.transform.location + carla.Location(z=0.3),
                size=GL_SZ, color=GL_COL,
                life_time=GL_LIFE, persistent_lines=False)
    except Exception:
        pass
