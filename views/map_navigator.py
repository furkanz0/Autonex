"""
views/map_navigator.py — Bird's-eye Map Navigator
════════════════════════════════════════════════════
• Displays the map from above (bird's-eye view) in a Pygame window
• Left click  → Select start point (GREEN)
• Right click → Select end point (RED)
• S           → Save screenshot
• ENTER       → Start test with selected route
• ESC         → Exit
• Scroll      → Zoom in / out
• Middle-click drag → Pan the map
"""

import os
import math
import time
import datetime

import numpy as np
import carla

from config import HAS_PYGAME, HAS_GRP
from utils.logger import log, sec

if HAS_PYGAME:
    import pygame

if HAS_GRP:
    from agents.navigation.global_route_planner import GlobalRoutePlanner


# ── Color Palette ────────────────────────────────────────────────────────
COL_BG        = (18, 18, 24)
COL_ROAD      = (55, 55, 70)
COL_SIDEWALK  = (40, 40, 50)
COL_START     = (0, 220, 80)
COL_END       = (220, 40, 40)
COL_ROUTE     = (80, 160, 255)
COL_SPAWN     = (255, 200, 50)
COL_TEXT      = (200, 200, 200)
COL_PANEL     = (30, 30, 40, 200)
COL_GRID      = (35, 35, 45)


class MapNavigator:
    """
    Interactive bird's-eye navigator for CARLA maps.

    Usage:
        nav = MapNavigator(world)
        result = nav.run()
        # result = {"start": carla.Location, "end": carla.Location} or None
    """

    WIN_W, WIN_H = 1280, 800
    MARGIN = 60
    POINT_R = 8
    SCREENSHOT_DIR = "screenshots"

    def __init__(self, world):
        if not HAS_PYGAME:
            raise RuntimeError("pygame required: pip install pygame")

        self.world = world
        self.wmap = world.get_map()

        # Map data
        self._topology = []      # (start_wp, end_wp) pairs
        self._spawn_pts = []     # Transform list
        self._road_segments = [] # pixel (x1,y1,x2,y2) list

        # Coordinate transform
        self._min_x = self._min_y = 0.0
        self._max_x = self._max_y = 0.0
        self._scale = 1.0

        # User selections (world coordinates)
        self.start_loc = None   # carla.Location
        self.end_loc = None     # carla.Location

        # Pan & Zoom
        self._zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._dragging = False
        self._drag_origin = (0, 0)
        self._pan_origin = (0.0, 0.0)

        # Route waypoints (pixels)
        self._route_pixels = []
        self._route_wps = []     # carla.Waypoint list

        # Prepare map data
        self._prepare_map_data()

        log("MapNavigator initialized")

    # ─── Map Data Preparation ────────────────────────────────────────────

    def _prepare_map_data(self):
        """Read topology and spawn points, compute bounding box."""
        sec("MapNavigator – Map Data")
        self._topology = self.wmap.get_topology()
        self._spawn_pts = self.wmap.get_spawn_points()

        # Collect all waypoint positions → bounding box
        all_x, all_y = [], []
        for wp_s, wp_e in self._topology:
            all_x.append(wp_s.transform.location.x)
            all_y.append(wp_s.transform.location.y)
            all_x.append(wp_e.transform.location.x)
            all_y.append(wp_e.transform.location.y)
        for sp in self._spawn_pts:
            all_x.append(sp.location.x)
            all_y.append(sp.location.y)

        self._min_x = min(all_x) - 20
        self._max_x = max(all_x) + 20
        self._min_y = min(all_y) - 20
        self._max_y = max(all_y) + 20

        range_x = self._max_x - self._min_x
        range_y = self._max_y - self._min_y
        draw_w = self.WIN_W - 2 * self.MARGIN
        draw_h = self.WIN_H - 2 * self.MARGIN
        self._scale = min(draw_w / range_x, draw_h / range_y)

        # Detail each topology segment with intermediate waypoints
        self._road_segments = []
        for wp_s, wp_e in self._topology:
            pts = self._interpolate_segment(wp_s, wp_e)
            for i in range(len(pts) - 1):
                self._road_segments.append(
                    (pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1]))

        log(f"Topology: {len(self._topology)} segments, "
            f"{len(self._road_segments)} lines, "
            f"{len(self._spawn_pts)} spawn points")

    def _interpolate_segment(self, wp_start, wp_end, step=2.0):
        """Detail the path from start → end with a waypoint chain."""
        pts = []
        cur = wp_start
        end_loc = wp_end.transform.location
        pts.append(self._world_to_base(cur.transform.location))
        for _ in range(500):
            nxt = cur.next(step)
            if not nxt:
                break
            cur = nxt[0]
            pts.append(self._world_to_base(cur.transform.location))
            if cur.transform.location.distance(end_loc) < step * 1.5:
                break
        pts.append(self._world_to_base(end_loc))
        return pts

    # ─── Coordinate Transforms ───────────────────────────────────────────

    def _world_to_base(self, loc):
        """Convert world coordinates to base pixel position (before zoom/pan)."""
        px = self.MARGIN + (loc.x - self._min_x) * self._scale
        py = self.MARGIN + (loc.y - self._min_y) * self._scale
        return (px, py)

    def _base_to_screen(self, bx, by):
        """Base pixel → screen pixel (apply zoom + pan)."""
        cx, cy = self.WIN_W / 2, self.WIN_H / 2
        sx = (bx - cx) * self._zoom + cx + self._pan_x
        sy = (by - cy) * self._zoom + cy + self._pan_y
        return (int(sx), int(sy))

    def _screen_to_base(self, sx, sy):
        """Screen pixel → base pixel (reverse zoom + pan)."""
        cx, cy = self.WIN_W / 2, self.WIN_H / 2
        bx = (sx - cx - self._pan_x) / self._zoom + cx
        by = (sy - cy - self._pan_y) / self._zoom + cy
        return (bx, by)

    def _screen_to_world(self, sx, sy):
        """Convert screen pixel to world coordinates."""
        bx, by = self._screen_to_base(sx, sy)
        wx = (bx - self.MARGIN) / self._scale + self._min_x
        wy = (by - self.MARGIN) / self._scale + self._min_y
        return carla.Location(x=wx, y=wy, z=0.0)

    # ─── Route Computation ───────────────────────────────────────────────

    def _compute_route(self):
        """Compute route between selected start and end."""
        if not self.start_loc or not self.end_loc:
            self._route_pixels = []
            self._route_wps = []
            return

        try:
            from models.route import build_route, snap_directed, pick_spawn
            
            # Snap start location to the actual spawn point the simulation will use
            spawn_tf = pick_spawn(self.wmap, self.start_loc, self.end_loc)
            if spawn_tf:
                aligned_start = spawn_tf.location
                # Optionally, update the visual start_loc so the green circle matches the spawn point
                self.start_loc = aligned_start
            else:
                aligned_start = snap_directed(self.wmap, self.start_loc, self.end_loc)
                
            aligned_end = snap_directed(self.wmap, self.end_loc, self.start_loc)
            
            self._route_wps = build_route(self.wmap, aligned_start, aligned_end, self.world)
            self._route_pixels = []
            for wp in self._route_wps:
                self._route_pixels.append(
                    self._world_to_base(wp.transform.location))
            log(f"MapNavigator route computed: {len(self._route_wps)} waypoints")
        except Exception as e:
            log(f"Route compute error: {e}", "!")
            # Fallback: straight line
            self._route_pixels = [
                self._world_to_base(self.start_loc),
                self._world_to_base(self.end_loc),
            ]
            self._route_wps = []

    # ─── Screenshot ──────────────────────────────────────────────────────

    def _save_screenshot(self, surface):
        """Save the current Pygame surface as a PNG file."""
        os.makedirs(self.SCREENSHOT_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.SCREENSHOT_DIR, f"map_{ts}.png")
        pygame.image.save(surface, path)
        log(f"Screenshot saved: {path}")
        return path

    # ─── Drawing ─────────────────────────────────────────────────────────

    def _draw(self, surface, font, font_sm):
        """Draw the map, points and HUD."""
        surface.fill(COL_BG)

        # Grid
        for gx in range(0, self.WIN_W, 80):
            sx, _ = self._base_to_screen(gx, 0)
            pygame.draw.line(surface, COL_GRID, (sx, 0), (sx, self.WIN_H), 1)
        for gy in range(0, self.WIN_H, 80):
            _, sy = self._base_to_screen(0, gy)
            pygame.draw.line(surface, COL_GRID, (0, sy), (self.WIN_W, sy), 1)

        # Road segments
        lw = max(2, int(3 * self._zoom))
        for x1, y1, x2, y2 in self._road_segments:
            s1 = self._base_to_screen(x1, y1)
            s2 = self._base_to_screen(x2, y2)
            pygame.draw.line(surface, COL_ROAD, s1, s2, lw)

        # Spawn points
        sr = max(2, int(3 * self._zoom))
        for sp in self._spawn_pts:
            bp = self._world_to_base(sp.location)
            sp_scr = self._base_to_screen(bp[0], bp[1])
            pygame.draw.circle(surface, COL_SPAWN, sp_scr, sr)

        # Route line
        if len(self._route_pixels) > 1:
            scr_pts = [self._base_to_screen(p[0], p[1])
                       for p in self._route_pixels]
            rlw = max(2, int(4 * self._zoom))
            pygame.draw.lines(surface, COL_ROUTE, False, scr_pts, rlw)

        # Start point
        if self.start_loc:
            bp = self._world_to_base(self.start_loc)
            sp = self._base_to_screen(bp[0], bp[1])
            pr = max(6, int(self.POINT_R * self._zoom))
            pygame.draw.circle(surface, COL_START, sp, pr)
            pygame.draw.circle(surface, (255, 255, 255), sp, pr, 2)
            lbl = font_sm.render("START", True, COL_START)
            surface.blit(lbl, (sp[0] + pr + 4, sp[1] - 8))

        # End point
        if self.end_loc:
            bp = self._world_to_base(self.end_loc)
            ep = self._base_to_screen(bp[0], bp[1])
            pr = max(6, int(self.POINT_R * self._zoom))
            pygame.draw.circle(surface, COL_END, ep, pr)
            pygame.draw.circle(surface, (255, 255, 255), ep, pr, 2)
            lbl = font_sm.render("END", True, COL_END)
            surface.blit(lbl, (ep[0] + pr + 4, ep[1] - 8))

        # HUD Panel
        self._draw_hud(surface, font, font_sm)

    def _draw_hud(self, surface, font, font_sm):
        """Draw the info panel."""
        panel = pygame.Surface((280, 260), pygame.SRCALPHA)
        panel.fill(COL_PANEL)
        surface.blit(panel, (10, 10))

        lines = [
            ("MAP NAVIGATOR", COL_ROUTE),
            ("", COL_TEXT),
            ("Left Click  -> Start", COL_START),
            ("Right Click -> End", COL_END),
            ("Scroll      -> Zoom", COL_TEXT),
            ("Mid Drag    -> Pan", COL_TEXT),
            ("S           -> Screenshot", COL_TEXT),
            ("ENTER       -> Run Test", COL_ROUTE),
            ("ESC         -> Exit", (150, 150, 150)),
        ]
        y = 18
        for txt, col in lines:
            if txt:
                ts = font_sm.render(txt, True, col)
                surface.blit(ts, (20, y))
            y += 24

        # Selected coordinates
        y += 4
        if self.start_loc:
            t = f"Start: ({self.start_loc.x:.0f}, {self.start_loc.y:.0f})"
            surface.blit(font_sm.render(t, True, COL_START), (20, y))
        y += 20
        if self.end_loc:
            t = f"End:   ({self.end_loc.x:.0f}, {self.end_loc.y:.0f})"
            surface.blit(font_sm.render(t, True, COL_END), (20, y))

        # Zoom info
        zt = font_sm.render(f"Zoom: {self._zoom:.1f}x", True, (120, 120, 120))
        surface.blit(zt, (self.WIN_W - 120, self.WIN_H - 30))

    # ─── Snap to Road ────────────────────────────────────────────────────

    def _snap_to_road(self, loc):
        """Snap the clicked point to the nearest driving lane."""
        wp = self.wmap.get_waypoint(
            loc, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        if wp:
            return wp.transform.location
        return loc

    # ─── Main Loop ───────────────────────────────────────────────────────

    def run(self):
        """
        Open the Pygame navigator window.

        Returns:
            dict  → {"start": carla.Location, "end": carla.Location,
                      "waypoints": [carla.Waypoint, ...]}
            None  → User exited with ESC
        """
        sec("MapNavigator – Opening Window")
        pygame.init()
        pygame.display.set_caption("Autonex Map Navigator — Town04")
        screen = pygame.display.set_mode((self.WIN_W, self.WIN_H))
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("Consolas", 18, bold=True)
        font_sm = pygame.font.SysFont("Consolas", 15)

        running = True
        result = None

        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False

                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key == pygame.K_s:
                        self._save_screenshot(screen)
                    elif ev.key == pygame.K_RETURN:
                        if self.start_loc and self.end_loc:
                            result = {
                                "start": self.start_loc,
                                "end": self.end_loc,
                                "waypoints": self._route_wps,
                            }
                            running = False
                        else:
                            log("Please select both start and end points!", "!")

                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    if ev.button == 1:  # Left click → start
                        wloc = self._screen_to_world(*ev.pos)
                        self.start_loc = self._snap_to_road(wloc)
                        log(f"Start: ({self.start_loc.x:.1f}, "
                            f"{self.start_loc.y:.1f})")
                        self._compute_route()

                    elif ev.button == 3:  # Right click → end
                        wloc = self._screen_to_world(*ev.pos)
                        self.end_loc = self._snap_to_road(wloc)
                        log(f"End: ({self.end_loc.x:.1f}, "
                            f"{self.end_loc.y:.1f})")
                        self._compute_route()

                    elif ev.button == 2:  # Middle click → start drag
                        self._dragging = True
                        self._drag_origin = ev.pos
                        self._pan_origin = (self._pan_x, self._pan_y)

                    elif ev.button == 4:  # Scroll up → zoom in
                        self._zoom = min(self._zoom * 1.15, 20.0)

                    elif ev.button == 5:  # Scroll down → zoom out
                        self._zoom = max(self._zoom / 1.15, 0.2)

                elif ev.type == pygame.MOUSEBUTTONUP:
                    if ev.button == 2:
                        self._dragging = False

                elif ev.type == pygame.MOUSEMOTION:
                    if self._dragging:
                        dx = ev.pos[0] - self._drag_origin[0]
                        dy = ev.pos[1] - self._drag_origin[1]
                        self._pan_x = self._pan_origin[0] + dx
                        self._pan_y = self._pan_origin[1] + dy

            self._draw(screen, font, font_sm)
            pygame.display.flip()
            clock.tick(30)

        pygame.quit()
        return result
