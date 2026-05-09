"""
views/minimap.py — OpenCV-based dynamic Live Minimap
"""

import cv2
import numpy as np
import carla
import math
from utils.logger import log

class MiniMap:
    def __init__(self, world, waypoints, win_size=800, margin=40):
        self.world = world
        self.wmap = world.get_map()
        self.win_size = win_size
        self.margin = margin
        
        self._min_x = self._min_y = 0.0
        self._max_x = self._max_y = 0.0
        self._scale = 1.0
        
        self.bg_map = np.zeros((win_size, win_size, 3), dtype=np.uint8)
        self.bg_map.fill(18)  # Dark background (BGR: 18, 18, 18)
        
        log("Initializing Dynamic Minimap...")
        self._prepare_map(waypoints)

    def _prepare_map(self, waypoints):
        """Builds the base static map image with road topology and full path."""
        topology = self.wmap.get_topology()
        
        # Calculate bounding box to dynamically scale the map
        all_x, all_y = [], []
        for wp_s, wp_e in topology:
            all_x.extend([wp_s.transform.location.x, wp_e.transform.location.x])
            all_y.extend([wp_s.transform.location.y, wp_e.transform.location.y])
            
        self._min_x = min(all_x) - 20
        self._max_x = max(all_x) + 20
        self._min_y = min(all_y) - 20
        self._max_y = max(all_y) + 20
        
        range_x = self._max_x - self._min_x
        range_y = self._max_y - self._min_y
        draw_w = self.win_size - 2 * self.margin
        draw_h = self.win_size - 2 * self.margin
        
        self._scale = min(draw_w / range_x, draw_h / range_y)
        
        # 1. Draw road segments dynamically based on topology
        road_color = (55, 55, 55) # BGR
        for wp_s, wp_e in topology:
            self._draw_road_segment(wp_s, wp_e, road_color)
            
        # 2. Draw actual full route path (Distance/Path, not Displacement)
        if waypoints and len(waypoints) > 1:
            route_pts = []
            for wp in waypoints:
                px, py = self._world_to_pixel(wp.transform.location.x, wp.transform.location.y)
                route_pts.append([px, py])
                
            route_pts = np.array(route_pts, np.int32)
            route_pts = route_pts.reshape((-1, 1, 2))
            
            # Thick blue line for the route
            route_color = (255, 160, 80) # OpenCV uses BGR
            cv2.polylines(self.bg_map, [route_pts], isClosed=False, color=route_color, thickness=4, lineType=cv2.LINE_AA)
            
            # Start (Green) and End (Red) markers
            start_pt = tuple(route_pts[0][0])
            end_pt = tuple(route_pts[-1][0])
            cv2.circle(self.bg_map, start_pt, 8, (80, 220, 0), -1, cv2.LINE_AA)
            cv2.circle(self.bg_map, end_pt, 8, (40, 40, 220), -1, cv2.LINE_AA)

    def _draw_road_segment(self, wp_start, wp_end, color, step=2.0):
        """Draw a single road segment accurately by interpolating intermediate waypoints."""
        pts = []
        cur = wp_start
        end_loc = wp_end.transform.location
        px, py = self._world_to_pixel(cur.transform.location.x, cur.transform.location.y)
        pts.append([px, py])
        
        for _ in range(500):
            nxt = cur.next(step)
            if not nxt:
                break
            cur = nxt[0]
            px, py = self._world_to_pixel(cur.transform.location.x, cur.transform.location.y)
            pts.append([px, py])
            if cur.transform.location.distance(end_loc) < step * 1.5:
                break
        
        px, py = self._world_to_pixel(end_loc.x, end_loc.y)
        pts.append([px, py])
        
        pts = np.array(pts, np.int32).reshape((-1, 1, 2))
        cv2.polylines(self.bg_map, [pts], isClosed=False, color=color, thickness=2, lineType=cv2.LINE_AA)

    def _world_to_pixel(self, x, y):
        """Transforms 3D World coordinates to 2D Pixel coordinates."""
        px = self.margin + (x - self._min_x) * self._scale
        py = self.margin + (y - self._min_y) * self._scale
        return int(px), int(py)

    def render(self, vehicle_transform):
        """Updates the live map with the current vehicle location."""
        # Copy the pre-drawn static background
        canvas = self.bg_map.copy()
        
        # Vehicle coordinates
        v_loc = vehicle_transform.location
        v_rot = vehicle_transform.rotation
        
        vx, vy = self._world_to_pixel(v_loc.x, v_loc.y)
        
        # Draw vehicle as a red circle
        cv2.circle(canvas, (vx, vy), 6, (0, 0, 255), -1, cv2.LINE_AA) 
        
        # Draw heading indicator (small arrow/line pointing where the car is facing)
        yaw_rad = math.radians(v_rot.yaw)
        length = 15
        end_x = int(vx + length * math.cos(yaw_rad))
        end_y = int(vy + length * math.sin(yaw_rad))
        cv2.line(canvas, (vx, vy), (end_x, end_y), (0, 0, 255), 2, cv2.LINE_AA)
        
        # Overlay UI texts
        cv2.putText(canvas, "LIVE TRACKING MINIMAP", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"Vehicle Pos : X={v_loc.x:.1f}, Y={v_loc.y:.1f}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
        
        # Show window
        cv2.imshow("Mini-Map", canvas)
        cv2.waitKey(1)

    def destroy(self):
        """Closes the OpenCV window safely."""
        try:
            cv2.destroyWindow("Mini-Map")
        except Exception:
            pass
