"""
controllers/simulation.py — Main simulation loop (orchestration)

İki mod destekler:
  1. run()      — Waypoint tabanlı PID kontrol (varsayılan)
  2. run_lane() — Şerit tespit + CARLA harita PID kontrol (--lane)
"""

import time
import math
import ctypes
import carla

from config import TARGET_SPEED_KMH, GOAL_M, MAX_S, FIXED_DELTA
from utils.logger import sec, log
from views.spectator import spectator_update
from views.green_line import green_line
from controllers.vehicle_controller import compute_control


# =====================================================================
#  WAYPOINT MODE — run()
# =====================================================================

def run(world, vehicle, waypoints, wmap, end_loc, minimap=None):
    """
    Main simulation loop.
    Compute control → tick → update spectator/line → telemetry → goal/time check
    """
    sec("6 – Simulation Loop  (Proportional Waypoint Control)")
    frame    = 0
    t0       = time.time()
    wp_idx   = 0
    stall_t  = 0   # zero-speed counter

    # Spectator'ı başlangıçta araca taşı (sonrası serbest kontrol)
    spec = world.get_spectator()
    spectator_update(spec, vehicle)

    print(f"  Waypoints  : {len(waypoints)}")
    print(f"  Goal       : ({end_loc.x:.0f}, {end_loc.y:.0f})")
    print(f"  Target spd : {TARGET_SPEED_KMH} km/h")
    print()
    print(f"  {'F':>6}  {'t':>5}  {'km/h':>6}  {'dist':>7}  thr   br   st")
    print(f"  {'─'*6}  {'─'*5}  {'─'*6}  {'─'*7}  {'─'*5} {'─'*4} {'─'*5}")

    while True:
        tick_start = time.perf_counter()

        # ── Compute control ──────────────────────────────────────────
        ctrl, wp_idx = compute_control(vehicle, waypoints, wp_idx)
        vehicle.apply_control(ctrl)

        # ── Simulation step ──────────────────────────────────────────
        world.tick()
        frame += 1
        elapsed = time.time() - t0

        # ── Green line + kamera takibi ────────────────────────────────
        green_line(world, vehicle, waypoints, wp_idx)
        spectator_update(spec, vehicle)

        # ── Telemetry ────────────────────────────────────────────────
        try:
            vel  = vehicle.get_velocity()
            spd  = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            loc  = vehicle.get_location()
            dist = loc.distance(waypoints[wp_idx].transform.location)
            for i in range(wp_idx, len(waypoints) - 1):
                dist += waypoints[i].transform.location.distance(
                    waypoints[i + 1].transform.location
                )
        except RuntimeError:
            print("\n  [!] Vehicle unreachable")
            return {"ok": False, "f": frame, "t": elapsed}

        # ── MiniMap ──────────────────────────────────────────────────
        if minimap:
            minimap.render(vehicle.get_transform(), waypoints, wp_idx)

        # ── Log (every 40 frames) ────────────────────────────────────
        if frame % 40 == 0:
            print(f"  {frame:>6}  {elapsed:>4.1f}s  "
                  f"{spd:>5.1f}  {dist:>6.0f}m  "
                  f"{ctrl.throttle:.2f}  {ctrl.brake:.2f}  {ctrl.steer:+.2f}")

        # ── Route completed (all waypoints consumed) ─────────────────
        if wp_idx >= len(waypoints) - 1:
            # Brake and stop
            stop = carla.VehicleControl(throttle=0.0, brake=1.0)
            for _ in range(40):   # 2s braking
                vehicle.apply_control(stop)
                world.tick()
            print(f"\n  {'═'*55}")
            print(f"  [🏁] ROUTE COMPLETED!  {elapsed:.1f}s  {frame} frames")
            print(f"  {'═'*55}")
            _wait_before_close(world, vehicle, minimap, waypoints, wp_idx)
            if minimap:
                minimap.destroy()
            return {"ok": True, "f": frame, "t": elapsed}

        # ── Stall detector (5s) ──────────────────────────────────────
        if spd < 0.5:
            stall_t += 1
            if stall_t >= 100:   # 5s = 100 ticks × 0.05s
                print(f"\n  [!] {stall_t//20}s stalled! "
                      f"RAW THROTTLE override (2s)...")
                oc = carla.VehicleControl(throttle=0.8, steer=0.0,
                                          brake=0.0, hand_brake=False)
                for _ in range(40):
                    vehicle.apply_control(oc)
                    world.tick()
                stall_t = 0
                print("  [~] Override done, back to normal.")
        else:
            stall_t = 0

        # ── Goal distance ────────────────────────────────────────────
        if dist < GOAL_M:
            # Brake smoothly
            stop = carla.VehicleControl(throttle=0.0, brake=1.0)
            for _ in range(40):
                vehicle.apply_control(stop)
                world.tick()
            print(f"\n  {'═'*55}")
            print(f"  [🏁] GOAL REACHED!  {elapsed:.1f}s  {frame} frames")
            print(f"  {'═'*55}")
            _wait_before_close(world, vehicle, minimap, waypoints, wp_idx)
            if minimap:
                minimap.destroy()
            return {"ok": True, "f": frame, "t": elapsed}

        # ── Timeout ──────────────────────────────────────────────────
        if elapsed > MAX_S:
            print(f"\n  [!] Timeout ({MAX_S}s)")
            return {"ok": False, "f": frame, "t": elapsed}

        # ── Real-time sync (gerçek zamana senkronizasyon) ────────────
        tick_elapsed = time.perf_counter() - tick_start
        sleep_time = FIXED_DELTA - tick_elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    return {"ok": False, "f": frame, "t": time.time() - t0}


def _wait_before_close(world, vehicle, minimap=None, waypoints=None, wp_idx=0):
    """Keep simulation visible for 5 seconds after goal reached."""
    print("  [~] Waiting 5s before cleanup...")
    for _ in range(100):  # 5s = 100 × 0.05
        wt0 = time.perf_counter()
        world.tick()
        try:
            if minimap:
                minimap.render(vehicle.get_transform(), waypoints, wp_idx)
        except Exception:
            break
        wt_elapsed = time.perf_counter() - wt0
        wt_sleep = FIXED_DELTA - wt_elapsed
        if wt_sleep > 0:
            time.sleep(wt_sleep)


# =====================================================================
#  LANE FOLLOWING MODE — run_lane()
# =====================================================================

class LaneChangeKeyboard:
    """Windows global key polling for lane-change commands."""

    VK_LEFT = 0x25
    VK_RIGHT = 0x27
    VK_A = 0x41
    VK_D = 0x44
    VK_1 = 0x31
    VK_2 = 0x32
    VK_3 = 0x33
    VK_R = 0x52
    VK_G = 0x47

    def __init__(self):
        self._last_pressed = False
        self._last_command = None
        self._last_weather_pressed = False
        self._last_tl_pressed = False

    def poll(self):
        pressed_left = self._down(self.VK_A) or self._down(self.VK_LEFT)
        pressed_right = self._down(self.VK_D) or self._down(self.VK_RIGHT)
        pressed = pressed_left or pressed_right

        command = None
        if pressed and not self._last_pressed:
            if pressed_left:
                command = "left"
            elif pressed_right:
                command = "right"

        self._last_pressed = pressed
        self._last_command = command
        return command

    def poll_weather(self):
        pressed_1 = self._down(self.VK_1)
        pressed_2 = self._down(self.VK_2)
        pressed_3 = self._down(self.VK_3)
        pressed = pressed_1 or pressed_2 or pressed_3

        weather = None
        if pressed and not self._last_weather_pressed:
            if pressed_1:
                weather = "sunny"
            elif pressed_2:
                weather = "rainy"
            elif pressed_3:
                weather = "snowy"

        self._last_weather_pressed = pressed
        return weather

    def poll_tl_override(self):
        pressed_r = self._down(self.VK_R)
        pressed_g = self._down(self.VK_G)
        pressed = pressed_r or pressed_g

        cmd = None
        if pressed and not self._last_tl_pressed:
            if pressed_r:
                cmd = "red"
            elif pressed_g:
                cmd = "green"

        self._last_tl_pressed = pressed
        return cmd

    @staticmethod
    def _down(vk):
        state = ctypes.windll.user32.GetAsyncKeyState(vk)
        return bool(state & 0x8000 or state & 0x0001)


def run_lane(world, vehicle, end_loc=None, initial_lane_change=None, client=None,
             waypoints=None, guide_route=False, minimap=None):
    """
    Lane-following simulation loop.

    The vehicle drives by detecting lane markings with OpenCV,
    not by following pre-computed waypoints.

    Args:
        world: CARLA world
        vehicle: spawned vehicle actor
        end_loc: optional goal location (if None, drives indefinitely)
        initial_lane_change: optional initial lane change direction
        client: CARLA client (for traffic manager)
        waypoints: optional route waypoints from MapNavigator/GlobalRoutePlanner.
        guide_route: when True, route waypoints take over steering. The default
            keeps the old camera-based lane-following mechanics and only uses
            map data around junctions.
        minimap: optional MiniMap instance for live tracking overlay.

    Returns:
        dict with ok, f (frames), t (time)
    """
    # Lazy imports — bu modüller sadece lane modunda gerekli
    from views.lane_camera import LaneCamera
    from views.lane_dashboard import LaneDashboard
    from models.lane_detector import LaneDetector
    from controllers.lane_controller import LaneController
    from controllers.traffic_light_controller import CameraTrafficLightDetector

    sec("6 – Simulation Loop  (Pure Vision Mode)")

    # ── Initialize components ────────────────────────────────────────
    cam = LaneCamera(world, vehicle)
    detector = LaneDetector()
    controller = LaneController()
    dashboard = LaneDashboard()
    tl_detector = CameraTrafficLightDetector()
    keyboard = LaneChangeKeyboard()
    spec = world.get_spectator()
    traffic_manager = None
    tm_lane_change = None
    route_lock = None

    if client is not None and not guide_route:
        try:
            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 8.0)
            traffic_manager.set_desired_speed(vehicle, 30.0)
            # Otonom kırmızı ışık frenini kapat — tamamen kamera tespitine bırak
            traffic_manager.ignore_lights_percentage(vehicle, 100)
            vehicle.set_autopilot(True, traffic_manager.get_port())
            if waypoints and len(waypoints) >= 2:
                route_lock = _TrafficManagerRouteLock(traffic_manager, vehicle, waypoints)
                route_lock.reload(world=world, force=True)
            log("Traffic Manager lane keeping enabled at 30 km/h")
        except Exception as exc:
            traffic_manager = None
            route_lock = None
            vehicle.set_autopilot(False)
            log(f"Traffic Manager unavailable, falling back to local PID: {exc}", "!")

    # Wait for first camera frame
    log("Waiting for camera frame...")
    for _ in range(10):
        tick_frame = world.tick()
        cam.wait_for_frame(tick_frame)
    if cam.frame is None:
        log("No camera frame received!", "!")

    # ── Initial throttle burst to get the vehicle moving ──────────
    if traffic_manager is None:
        log("Gentle initial roll...")
        kick = carla.VehicleControl(throttle=0.35, steer=0.0, brake=0.0)
        for _ in range(8):
            vehicle.apply_control(kick)
            tick_frame = world.tick()
            cam.wait_for_frame(tick_frame)

    if initial_lane_change:
        if traffic_manager is not None:
            if _safe_lane_change_target(world, vehicle, initial_lane_change) is None:
                log(f"Initial lane change rejected, no legal same-direction lane: {initial_lane_change}", "!")
            else:
                next_lane_change = _start_tm_lane_change(
                    traffic_manager, vehicle, initial_lane_change, world)
                if next_lane_change is not None:
                    tm_lane_change = next_lane_change
                    detector.reset()
                    if route_lock is not None:
                        route_lock.mark_manual_lane_change(world, next_lane_change)
                    log(f"Initial Traffic Manager lane change requested: {initial_lane_change}")
        elif controller.request_lane_change(initial_lane_change):
            detector.reset()
            log(f"Initial lane change requested: {initial_lane_change}")

    frame = 0
    t0 = time.time()
    stall_t = 0
    assist_frames = 0
    wp_idx = 0
    route_guided = bool(guide_route and waypoints and len(waypoints) >= 2)

    goal_str = f"({end_loc.x:.0f}, {end_loc.y:.0f})" if end_loc else "None (free drive)"
    mode_str = "ROUTE-GUIDED LANE (map route + camera overlay)" if route_guided else "TRAFFIC MANAGER LANE KEEPING (OpenCV overlay)"
    print(f"  Mode       : {mode_str}")
    print(f"  Goal       : {goal_str}")
    print(f"  Control    : {'Route lane-center PID' if route_guided else 'Traffic Manager at 30 km/h, local PID fallback'}")
    if route_guided:
        print(f"  Waypoints  : {len(waypoints)}")
    print()
    print(f"  {'F':>6}  {'t':>5}  {'km/h':>6}  {'off':>6}  {'curv':>7}  {'conf':>5}  {'steer':>6}  {'TL':>8}")
    print(f"  {'─'*6}  {'─'*5}  {'─'*6}  {'─'*6}  {'─'*7}  {'─'*5}  {'─'*6}  {'─'*8}")

    while True:
        # ── Get camera frame ─────────────────────────────────────────
        raw = cam.frame

        # ── Detect lanes (OpenCV) ─────────────────────────────────────
        lane_result = detector.process(raw)

        # ── Detect traffic lights (OpenCV HSV) ────────────────────────
        tl_result = tl_detector.process(raw)

        # ── Compute speed ────────────────────────────────────────────
        vel = vehicle.get_velocity()
        spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

        # ── Junction Check ───────────────────────────────────────────
        loc = vehicle.get_location()
        wp = world.get_map().get_waypoint(loc)
        in_junction = wp.is_junction if wp else False

        # ── Compute control ──────────────────────────────────────────
        # Kırmızı ışık varsa, Traffic Manager olsun ya da olmasın kameraya uy
        obey_camera_light = tl_result.should_stop

        if traffic_manager is not None:
            prev_tm_lane_change = tm_lane_change
            tm_lane_change = _update_tm_lane_change(
                traffic_manager, vehicle, tm_lane_change, world)
            if prev_tm_lane_change is not None and tm_lane_change is None:
                detector.reset()
                if route_lock is not None:
                    route_lock.accept_current_lane(world)
                    route_lock.reload(world=world, force=True, preserve_locked_lane=True)
            if route_lock is not None:
                try:
                    route_lock.update(world, frame, tm_lane_change is not None)
                except Exception as exc:
                    log(f"Route lock update skipped this frame: {exc}", "!")

        if obey_camera_light:
            # Kırmızı ışık — fren uygula, mevcut direksiyonu koru
            brake_val = tl_result.brake_intensity()
            last_steer = controller.last_pid.get("steer", 0.0)
            if spd < 1.0:
                ctrl = carla.VehicleControl(
                    throttle=0.0, brake=0.5,
                    steer=float(last_steer))
            else:
                ctrl = carla.VehicleControl(
                    throttle=0.0, brake=float(brake_val),
                    steer=float(last_steer))
            
            # TM devredeyse, TM'nin oluşturduğu control'ü eziyoruz
            vehicle.apply_control(ctrl)
        elif traffic_manager is not None:
            ctrl = vehicle.get_control()
        else:
            if route_guided:
                # Keep the selected map route; camera processing still feeds the dashboard/overlay.
                ctrl, wp_idx = controller.compute_route(vehicle, spd, waypoints, wp_idx)
            else:
                # Local fallback when Traffic Manager is unavailable.
                ctrl = controller.compute_map(world, vehicle, spd)
            if assist_frames > 0:
                ctrl.throttle = max(float(ctrl.throttle), 0.45)
                ctrl.brake = 0.0
                assist_frames -= 1
            vehicle.apply_control(ctrl)

        # ── Tick ─────────────────────────────────────────────────────
        tick_frame = world.tick()
        cam.wait_for_frame(tick_frame)
        frame += 1
        elapsed = time.time() - t0

        # ── Spectator ────────────────────────────────────────────────
        spectator_update(spec, vehicle)
        try:
            debug_wp = _current_driving_waypoint(world, vehicle) or controller.debug_lane_waypoint
            _draw_carla_lane_debug(world, vehicle, debug_wp)
        except Exception as exc:
            log(f"Lane debug draw skipped this frame: {exc}", "!")

        # ── Dashboard ────────────────────────────────────────────────
        lane_state_ui = "JUNCTION" if (not obey_camera_light and in_junction) else controller.lane_change_state
        if not dashboard.render(
                lane_result, spd, ctrl.steer, frame,
                lane_state_ui, controller.target_offset_m,
                tl_state=tl_result.state,
                tl_confirmed=tl_result.confirmed):
            print("\n  [!] Window closed")
            vehicle.set_autopilot(False)
            _cleanup_lane(cam, dashboard, minimap)
            return {"ok": False, "f": frame, "t": elapsed}

        # ── MiniMap ──────────────────────────────────────────────────
        if minimap and waypoints:
            # Araç konumuna en yakın waypoint'i bul — rota çizgisi
            # sadece kalan kısmı göstersin
            minimap_idx = wp_idx
            if route_lock is not None:
                minimap_idx = route_lock.wp_idx
            elif not route_guided:
                best_dist = float("inf")
                for i, w in enumerate(waypoints):
                    d = loc.distance(w.transform.location)
                    if d < best_dist:
                        best_dist = d
                        minimap_idx = i
            minimap.render(vehicle.get_transform(), waypoints, minimap_idx)

        command = dashboard.consume_command() or keyboard.poll()
        if command:
            if traffic_manager is not None:
                if route_lock is not None and route_lock.blocks_lane_change(world):
                    log(f"Lane change held near junction to keep selected route: {command}", "!")
                elif _safe_lane_change_target(world, vehicle, command) is None:
                    log(f"Lane change rejected, no legal same-direction lane: {command}", "!")
                else:
                    next_lane_change = _start_tm_lane_change(
                        traffic_manager, vehicle, command, world)
                    if next_lane_change is not None:
                        tm_lane_change = next_lane_change
                        detector.reset()
                        if route_lock is not None:
                            route_lock.mark_manual_lane_change(world, next_lane_change)
                        log(f"Traffic Manager lane change started: {command}")
            elif controller.request_lane_change(command):
                detector.reset()
                log(f"Lane change requested: {command}")

        weather_command = dashboard.consume_weather() or keyboard.poll_weather()
        if weather_command:
            _apply_weather(world, weather_command)

        tl_override = dashboard.consume_tl_override() or keyboard.poll_tl_override()
        if tl_override:
            state = carla.TrafficLightState.Red if tl_override == "red" else carla.TrafficLightState.Green
            count = 0
            for actor in world.get_actors().filter('traffic.traffic_light'):
                actor.set_state(state)
                actor.freeze(True)
                count += 1
            log(f"MANUAL OVERRIDE: {count} Traffic Lights set to {state.name} and frozen.")

        # ── Console log (every 20 frames) ────────────────────────────
        if frame % 20 == 0:
            off = lane_result.lateral_offset_m
            curv = lane_result.curvature_m
            conf = lane_result.confidence * 100
            det = "✓" if lane_result.detected else "✗"

            tl_txt = ""
            if tl_result.state == "red":
                tl_txt = "🔴 RED" + (" STOP" if tl_result.confirmed else "")
            elif tl_result.state == "green":
                tl_txt = "🟢 GREEN"

            print(f"  {frame:>6}  {elapsed:>4.1f}s  "
                  f"{spd:>5.1f}  {off:>+5.2f}m  "
                  f"{curv:>6.0f}m  {conf:>4.0f}%  "
                  f"{ctrl.steer:+.3f}  {tl_txt:<8s}  "
                  f"{controller.lane_change_state:<6s}  {det}")

        # ── Goal check ───────────────────────────────────────────────
        if end_loc:
            loc = vehicle.get_location()
            dist = loc.distance(end_loc)
            route_done = route_guided and wp_idx >= len(waypoints) - 1
            tm_route_done = route_lock is not None and route_lock.is_route_done()
            free_drive_goal = route_lock is None and not route_guided and dist < GOAL_M
            if route_done or tm_route_done or free_drive_goal:
                vehicle.set_autopilot(False)
                stop = carla.VehicleControl(throttle=0.0, brake=1.0)
                for _ in range(40):
                    vehicle.apply_control(stop)
                    world.tick()
                print(f"\n  {'═'*55}")
                print(f"  [🏁] GOAL REACHED!  {elapsed:.1f}s  {frame} frames")
                print(f"  {'═'*55}")
                _cleanup_lane(cam, dashboard, minimap)
                return {"ok": True, "f": frame, "t": elapsed}

        # ── Stall detector ───────────────────────────────────────────
        if obey_camera_light:
            stall_t = 0  # kırmızı ışıkta durma stall değil
        elif spd < 0.5:
            stall_t += 1
            if stall_t >= 40:
                print(f"\n  [!] {stall_t//20}s stalled! low-speed assist armed...")
                assist_frames = 20
                stall_t = 0
        else:
            stall_t = 0

        # ── Timeout ──────────────────────────────────────────────────
        if elapsed > MAX_S:
            print(f"\n  [!] Timeout ({MAX_S}s)")
            vehicle.set_autopilot(False)
            _cleanup_lane(cam, dashboard, minimap)
            return {"ok": False, "f": frame, "t": elapsed}

    vehicle.set_autopilot(False)
    _cleanup_lane(cam, dashboard, minimap)
    return {"ok": False, "f": frame, "t": time.time() - t0}


# =====================================================================
#  LANE MODE YARDIMCI FONKSİYONLAR
# =====================================================================

def _cleanup_lane(cam, dashboard, minimap=None):
    """Destroy camera, dashboard, and optional minimap."""
    try:
        cam.destroy()
    except Exception:
        pass
    try:
        dashboard.destroy()
    except Exception:
        pass
    if minimap:
        try:
            minimap.destroy()
        except Exception:
            pass


def _set_tm_path(traffic_manager, vehicle, waypoints):
    path = []
    last_loc = None

    current_loc = vehicle.get_location()
    path.append(carla.Location(current_loc.x, current_loc.y, current_loc.z))
    last_loc = current_loc

    for wp in waypoints:
        loc = _route_point_location(wp)
        if loc is None:
            continue
        if last_loc is not None and loc.distance(last_loc) < 2.0:
            continue
        path.append(carla.Location(loc.x, loc.y, loc.z))
        last_loc = loc

    if len(path) < 2:
        return

    try:
        traffic_manager.set_path(vehicle, path, True)
        log(f"Traffic Manager route path loaded: {len(path)} points")
    except Exception as exc:
        log(f"Traffic Manager route path could not be loaded: {exc}", "!")


def _route_point_location(point):
    if isinstance(point, carla.Location):
        return point
    transform = getattr(point, "transform", None)
    if transform is not None:
        return transform.location
    return None


class _TrafficManagerRouteLock:
    """Keep Traffic Manager attached to the user-selected map route."""

    ROUTE_RETURN_DISTANCE_M = 20.0
    ROUTE_BLOCK_LANE_CHANGE_DISTANCE_M = 20.0
    ROUTE_RETURN_COMPLETE_BEFORE_M = 15.0

    def __init__(self, traffic_manager, vehicle, waypoints):
        self.traffic_manager = traffic_manager
        self.vehicle = vehicle
        self.waypoints = waypoints or []
        self.wp_idx = 0
        self._last_reload_frame = -999
        self._last_lane_change_active = False
        self._locked_road_id = None
        self._locked_section_id = None
        self._locked_lane_id = None
        self._accept_lane_after_manual_change = False
        self._manual_lane_change_pending = False
        self._manual_start_road_id = None
        self._manual_start_section_id = None
        self._manual_start_lane_id = None
        self._manual_pending_frames = 0
        self._manual_lane_road_id = None
        self._manual_lane_section_id = None
        self._manual_lane_id = None
        self._last_path_key = None
        self._active_reference_path = []
        self._last_lane_correction_frame = -999
        self._post_maneuver_grace_frames = 0
        self._last_wp_yaw = None
        self._route_return_active = False

    def update(self, world, frame, lane_change_active):
        if len(self.waypoints) < 2:
            return

        self._disable_uncommanded_lane_changes()

        loc = self.vehicle.get_location()
        current_wp = self._current_waypoint(world, loc)
        self.wp_idx = self._nearest_index(loc)
        self._update_post_maneuver_grace(current_wp)

        if current_wp is not None and not lane_change_active:
            if self._manual_lane_change_pending:
                self._update_manual_lane_change_lock(current_wp)
            elif self._manual_lane_id is None:
                self._refresh_lane_lock(current_wp)

        preserve_locked_lane = not self._manual_lane_change_pending
        in_recovery = self._post_maneuver_grace_frames > 0
        off_route = self._off_selected_route(world, loc, self.wp_idx)

        route_lane_wp = None
        in_route_window = self._inside_route_control_window()
        if current_wp is not None and in_route_window:
            self._clear_manual_lane()
            lane_change_active = False

        if current_wp is not None and not lane_change_active:
            route_lane_wp = self._required_route_lane_before_maneuver(current_wp)
            if route_lane_wp is not None and self._needs_route_lane_return(current_wp, route_lane_wp):
                self._clear_manual_lane()
                self._lock_to(route_lane_wp)
                if not self._route_return_active:
                    self._begin_route_return()
                    self.reload(
                        world=world,
                        force=True,
                        frame=frame,
                        preserve_locked_lane=True,
                        smooth_correction=True,
                    )
                    self._last_lane_correction_frame = frame
                self._last_lane_change_active = lane_change_active
                return
            elif self._route_return_active and not off_route and not in_route_window:
                self._finish_route_return()

        if off_route and not lane_change_active:
            if frame - self._last_lane_correction_frame >= 25:
                if current_wp is not None:
                    recovery_wp = self._nearest_route_lane_for_recovery(current_wp)
                    if recovery_wp is not None:
                        self._clear_manual_lane()
                        self._lock_to(recovery_wp)
                        self._begin_route_return()
                self.reload(
                    world=world,
                    force=True,
                    frame=frame,
                    preserve_locked_lane=True,
                    smooth_correction=True,
                )
                self._last_lane_correction_frame = frame
            self._last_lane_change_active = lane_change_active
            return

        if in_recovery and route_lane_wp is None:
            self._last_lane_change_active = lane_change_active
            return

        needs_reload = False
        if preserve_locked_lane and not lane_change_active and self._near_route_maneuver(self.wp_idx):
            needs_reload = True
        if off_route and not lane_change_active:
            needs_reload = True
        if self._last_lane_change_active and not lane_change_active:
            needs_reload = True
        if current_wp is not None and preserve_locked_lane and not lane_change_active:
            if (not self._route_return_active and
                    self._uncommanded_lane_change_detected(current_wp) and
                    self._should_correct_uncommanded_lane(current_wp)):
                if frame - self._last_lane_correction_frame >= 50:
                    self.reload(
                        world=world,
                        force=True,
                        frame=frame,
                        preserve_locked_lane=True,
                        smooth_correction=True,
                    )
                    self._last_lane_correction_frame = frame
                self._last_lane_change_active = lane_change_active
                return

        self._last_lane_change_active = lane_change_active
        if needs_reload and frame - self._last_reload_frame >= 60:
            self.reload(world=world, frame=frame, preserve_locked_lane=preserve_locked_lane)

    def blocks_lane_change(self, world):
        if len(self.waypoints) < 2:
            return False
        loc = self.vehicle.get_location()
        current_wp = self._current_waypoint(world, loc)
        if current_wp is None:
            return False
        if current_wp.is_junction:
            return True
        maneuver_distance = self._distance_to_route_maneuver(self.wp_idx)
        return (
            maneuver_distance is not None and
            maneuver_distance <= self.ROUTE_BLOCK_LANE_CHANGE_DISTANCE_M
        )

    def _inside_route_control_window(self):
        maneuver_distance = self._distance_to_route_maneuver(self.wp_idx)
        return (
            maneuver_distance is not None and
            maneuver_distance <= self.ROUTE_BLOCK_LANE_CHANGE_DISTANCE_M
        )

    def is_route_done(self):
        if len(self.waypoints) < 2:
            return False
        loc = self.vehicle.get_location()
        last_loc = self.waypoints[-1].transform.location
        near_end = loc.distance(last_loc) < GOAL_M
        progressed_to_end = self.wp_idx >= max(0, len(self.waypoints) - 6)
        return near_end and progressed_to_end

    def debug_route_waypoint(self, world=None):
        if not self.waypoints:
            return None
        idx = min(max(self.wp_idx, 0), len(self.waypoints) - 1)
        route_wp = self.waypoints[idx]
        current_wp = self._current_waypoint(world, self.vehicle.get_location())

        if world is not None and current_wp is not None:
            try:
                if (self._manual_lane_change_pending or self._manual_lane_id is not None):
                    if current_wp.lane_type == carla.LaneType.Driving:
                        return current_wp
                lane_id = self._manual_lane_id or self._locked_lane_id
                if lane_id is not None:
                    seed_wp = self._project_current_wp_to_lane(world, current_wp, lane_id)
                    if seed_wp is not None:
                        return seed_wp
                if current_wp.lane_type == carla.LaneType.Driving:
                    return current_wp
            except Exception:
                pass

        return route_wp

    def debug_route_points(self, world=None):
        """Return the exact active reference path currently loaded into TM."""
        if self._active_reference_path:
            loc = self.vehicle.get_location()
            start = self._nearest_point_index(self._active_reference_path, loc)
            start = max(0, start - 1)
            return self._active_reference_path[start:start + 28]

        if world is None:
            return None

        current_wp = self._current_waypoint(world, self.vehicle.get_location())
        if current_wp is None:
            return None

        remaining = self.waypoints[self.wp_idx:self.wp_idx + 28]
        if self._manual_lane_id is not None:
            remaining = self._temporary_lane_until_route_maneuver(world, current_wp, remaining)
        elif self._locked_lane_id is not None:
            remaining = self._same_lane_until_junction(world, current_wp, remaining)
        return remaining

    def _nearest_point_index(self, points, loc):
        best_idx = 0
        best_dist = float("inf")
        for i, point in enumerate(points):
            point_loc = _route_point_location(point)
            if point_loc is None:
                continue
            dist = loc.distance(point_loc)
            if dist < best_dist:
                best_idx = i
                best_dist = dist
        return best_idx

    def _manual_lane_debug_waypoint(self, world, current_wp, route_wp):
        if current_wp is None or self._manual_lane_id is None:
            return None
        if self._route_point_is_close_to_maneuver(route_wp):
            return None

        if (current_wp.road_id == self._manual_lane_road_id and
                current_wp.section_id == self._manual_lane_section_id and
                current_wp.lane_id == self._manual_lane_id):
            return current_wp

        try:
            wp = world.get_map().get_waypoint_xodr(
                self._manual_lane_road_id,
                self._manual_lane_id,
                current_wp.s,
            )
        except Exception:
            return None

        if wp is None or wp.lane_type != carla.LaneType.Driving:
            return None
        return wp

    def _project_current_wp_to_lane(self, world, current_wp, lane_id):
        if current_wp is None or lane_id is None:
            return None
        if current_wp.lane_id == lane_id:
            return current_wp
        try:
            wp = world.get_map().get_waypoint_xodr(
                current_wp.road_id,
                lane_id,
                current_wp.s,
            )
        except Exception:
            return None
        if wp is None or wp.lane_type != carla.LaneType.Driving:
            return None
        return wp

    def mark_manual_lane_change(self, world, lane_change_task=None):
        self._accept_lane_after_manual_change = True
        self._manual_lane_change_pending = True
        self._manual_pending_frames = 0
        if lane_change_task is not None:
            self._manual_lane_road_id = lane_change_task.get("target_road_id")
            self._manual_lane_section_id = lane_change_task.get("target_section_id")
            self._manual_lane_id = lane_change_task.get("target_lane_id")
        current_wp = self._current_waypoint(world, self.vehicle.get_location())
        if current_wp is not None:
            self._manual_start_road_id = current_wp.road_id
            self._manual_start_section_id = current_wp.section_id
            self._manual_start_lane_id = current_wp.lane_id
        self.reload(world=world, force=True, preserve_locked_lane=True)

    def accept_current_lane(self, world):
        current_wp = self._current_waypoint(world, self.vehicle.get_location())
        if current_wp is None or current_wp.is_junction:
            return
        self._manual_lane_road_id = current_wp.road_id
        self._manual_lane_section_id = current_wp.section_id
        self._manual_lane_id = current_wp.lane_id
        self._lock_to(current_wp)
        self._accept_lane_after_manual_change = False
        self._manual_lane_change_pending = False
        self._manual_pending_frames = 0

    def reload(self, world=None, force=False, frame=None,
               preserve_locked_lane=True, smooth_correction=False):
        if len(self.waypoints) < 2:
            return
        if not force and frame is not None and frame - self._last_reload_frame < 60:
            return

        self._disable_uncommanded_lane_changes()

        loc = self.vehicle.get_location()
        current_wp = self._current_waypoint(world, loc) if world is not None else None
        if (current_wp is not None and not self._manual_lane_change_pending and
                self._manual_lane_id is None):
            self._refresh_lane_lock(current_wp)

        self.wp_idx = self._nearest_index(loc)
        remaining = self.waypoints[self.wp_idx:]
        if len(remaining) < 2 and self.wp_idx > 0:
            remaining = self.waypoints[self.wp_idx - 1:]

        if world is not None and current_wp is not None and preserve_locked_lane:
            remaining = self._route_lane_or_temporary_lane(world, current_wp, remaining)

        if smooth_correction and current_wp is not None:
            remaining = self._smooth_return_to_locked_lane(current_wp, remaining)

        path_key = self._path_key(remaining)
        if not force and path_key == self._last_path_key:
            return

        _set_tm_path(self.traffic_manager, self.vehicle, remaining)
        self._active_reference_path = list(remaining)
        self._last_path_key = path_key
        if frame is not None:
            self._last_reload_frame = frame

    def _disable_uncommanded_lane_changes(self):
        try:
            self.traffic_manager.auto_lane_change(self.vehicle, False)
            self.traffic_manager.random_left_lanechange_percentage(self.vehicle, 0)
            self.traffic_manager.random_right_lanechange_percentage(self.vehicle, 0)
        except Exception:
            pass

    def _begin_route_return(self):
        self._route_return_active = True

    def _finish_route_return(self):
        self._route_return_active = False

    def _nearest_index(self, loc):
        search_from = max(0, self.wp_idx - 12)
        search_to = min(len(self.waypoints), self.wp_idx + 80)
        if search_from >= search_to:
            return self.wp_idx

        best_i = self.wp_idx
        best_dist = float("inf")
        for i in range(search_from, search_to):
            dist = loc.distance(self.waypoints[i].transform.location)
            if dist < best_dist:
                best_i = i
                best_dist = dist
        return best_i

    def _near_route_junction(self, idx):
        ahead = self.waypoints[idx:min(len(self.waypoints), idx + 22)]
        return any(getattr(wp, "is_junction", False) for wp in ahead)

    def _near_route_maneuver(self, idx):
        return self._route_maneuver_index(idx) is not None

    def _route_maneuver_index(self, idx):
        if idx >= len(self.waypoints):
            return None

        start_wp = self.waypoints[idx]
        last_loc = start_wp.transform.location
        last_wp = start_wp
        distance = 0.0

        for i in range(idx + 1, min(len(self.waypoints), idx + 80)):
            wp = self.waypoints[i]
            loc = wp.transform.location
            distance += loc.distance(last_loc)
            last_loc = loc

            route_transition = (
                wp.road_id != last_wp.road_id
            )
            transition_turns = abs(
                self._yaw_delta(last_wp.transform.rotation.yaw, wp.transform.rotation.yaw)
            ) > 8.0
            route_decision = (
                getattr(wp, "is_junction", False) or
                self._has_route_branch(last_wp) or
                (route_transition and transition_turns)
            )
            if route_decision:
                return i if distance <= 150.0 else None
            last_wp = wp
        return None

    def _has_route_branch(self, wp):
        forward = wp.transform.get_forward_vector()
        for lookahead in (4.0, 8.0, 12.0, 16.0):
            try:
                next_wps = wp.next(lookahead)
            except Exception:
                continue
            if len(next_wps) <= 1:
                continue

            branches = []
            seen = set()
            for candidate in next_wps:
                if candidate.lane_type != carla.LaneType.Driving:
                    continue
                cand_forward = candidate.transform.get_forward_vector()
                if forward.x * cand_forward.x + forward.y * cand_forward.y < 0.55:
                    continue
                key = (
                    candidate.road_id,
                    candidate.section_id,
                    round(candidate.transform.rotation.yaw / 5.0) * 5,
                )
                if key in seen:
                    continue
                seen.add(key)
                branches.append(candidate)

            if len(branches) > 1:
                return True
        return False

    def _distance_to_route_maneuver(self, idx):
        maneuver_idx = self._route_maneuver_index(idx)
        if maneuver_idx is None:
            return None

        distance = 0.0
        last_loc = self.waypoints[idx].transform.location
        for wp in self.waypoints[idx + 1:maneuver_idx + 1]:
            loc = wp.transform.location
            distance += loc.distance(last_loc)
            last_loc = loc
        return distance

    def _yaw_delta(self, yaw_a, yaw_b):
        return (yaw_b - yaw_a + 180.0) % 360.0 - 180.0

    def _update_post_maneuver_grace(self, current_wp):
        if current_wp is None:
            return

        if current_wp.is_junction:
            self._post_maneuver_grace_frames = 45
            return

        if self._post_maneuver_grace_frames > 0:
            self._post_maneuver_grace_frames -= 1

    def _required_route_lane_before_maneuver(self, current_wp):
        if current_wp.is_junction:
            return None

        maneuver_idx = self._route_maneuver_index(self.wp_idx)
        if maneuver_idx is None:
            return None
        maneuver_distance = self._distance_to_route_maneuver(self.wp_idx)
        if maneuver_distance is None or maneuver_distance > self.ROUTE_RETURN_DISTANCE_M:
            return None

        route_lane_wp = self._route_lane_before_maneuver(current_wp, maneuver_idx)

        if route_lane_wp is None:
            return None

        dist = current_wp.transform.location.distance(route_lane_wp.transform.location)
        if dist > 135.0:
            return None
        if not self._same_direction_waypoints(current_wp, route_lane_wp):
            return None
        return route_lane_wp

    def _route_lane_before_maneuver(self, current_wp, maneuver_idx):
        candidates = []
        current_loc = current_wp.transform.location
        for wp in self.waypoints[self.wp_idx:maneuver_idx]:
            if getattr(wp, "is_junction", False):
                break
            if not self._same_direction_waypoints(current_wp, wp):
                continue
            dist = current_loc.distance(wp.transform.location)
            if dist > 80.0:
                continue
            same_segment = (
                wp.road_id == current_wp.road_id and
                wp.section_id == current_wp.section_id
            )
            score = (0 if same_segment else 25.0) + dist
            candidates.append((score, wp))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def _nearest_route_lane_for_recovery(self, current_wp):
        current_loc = current_wp.transform.location
        start = max(0, self.wp_idx - 8)
        stop = min(len(self.waypoints), self.wp_idx + 48)
        candidates = []
        for wp in self.waypoints[start:stop]:
            if getattr(wp, "is_junction", False):
                continue
            if not self._same_direction_waypoints(current_wp, wp):
                continue
            dist = current_loc.distance(wp.transform.location)
            if dist > 45.0:
                continue
            same_segment = (
                wp.road_id == current_wp.road_id and
                wp.section_id == current_wp.section_id
            )
            score = (0 if same_segment else 20.0) + dist
            candidates.append((score, wp))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def _needs_route_lane_return(self, current_wp, route_lane_wp):
        if current_wp.road_id != route_lane_wp.road_id:
            return True
        if current_wp.section_id != route_lane_wp.section_id:
            return True
        return current_wp.lane_id != route_lane_wp.lane_id

    def _should_correct_uncommanded_lane(self, current_wp):
        return self._required_route_lane_before_maneuver(current_wp) is not None

    def _current_waypoint(self, world, loc):
        if world is None:
            return None
        try:
            return world.get_map().get_waypoint(
                loc,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
        except Exception:
            return None

    def _refresh_lane_lock(self, current_wp):
        if current_wp.is_junction:
            return
        route_lane_wp = self._required_route_lane_before_maneuver(current_wp)
        if route_lane_wp is not None:
            self._lock_to(route_lane_wp)
            return
        if self._locked_road_id is None:
            self._lock_to(current_wp)
            return
        same_segment = (
            current_wp.road_id == self._locked_road_id and
            current_wp.section_id == self._locked_section_id
        )
        if not same_segment:
            self._lock_to(current_wp)

    def _reference_lane_for_current_segment(self, current_wp):
        search_from = max(0, self.wp_idx - 12)
        search_to = min(len(self.waypoints), self.wp_idx + 80)
        candidates = [
            wp for wp in self.waypoints[search_from:search_to]
            if wp.road_id == current_wp.road_id and
            wp.section_id == current_wp.section_id
        ]
        if not candidates:
            return None
        loc = current_wp.transform.location
        return min(candidates, key=lambda wp: loc.distance(wp.transform.location))

    def _lock_to(self, wp):
        self._locked_road_id = wp.road_id
        self._locked_section_id = wp.section_id
        self._locked_lane_id = wp.lane_id

    def _uncommanded_lane_change_detected(self, current_wp):
        if self._locked_lane_id is None or current_wp.is_junction:
            return False
        same_segment = (
            current_wp.road_id == self._locked_road_id and
            current_wp.section_id == self._locked_section_id
        )
        return same_segment and current_wp.lane_id != self._locked_lane_id

    def _update_manual_lane_change_lock(self, current_wp):
        if current_wp.is_junction:
            return

        self._manual_pending_frames += 1
        changed_lane = (
            current_wp.road_id != self._manual_start_road_id or
            current_wp.section_id != self._manual_start_section_id or
            current_wp.lane_id != self._manual_start_lane_id
        )
        timeout = self._manual_pending_frames >= 140

        if changed_lane or timeout:
            self._manual_lane_road_id = current_wp.road_id
            self._manual_lane_section_id = current_wp.section_id
            self._manual_lane_id = current_wp.lane_id
            self._lock_to(current_wp)
            self._accept_lane_after_manual_change = False
            self._manual_lane_change_pending = False
            self._manual_pending_frames = 0
            log(f"Manual lane change accepted temporarily: road={current_wp.road_id}, lane={current_wp.lane_id}")

    def _route_lane_or_temporary_lane(self, world, current_wp, remaining):
        route_lane_wp = self._required_route_lane_before_maneuver(current_wp)
        if route_lane_wp is not None:
            self._clear_manual_lane()
            self._lock_to(route_lane_wp)
            return self._same_lane_until_junction(world, current_wp, remaining)

        if self._manual_lane_id is not None:
            return self._temporary_lane_until_route_maneuver(world, current_wp, remaining)

        return self._same_lane_until_junction(world, current_wp, remaining)

    def _same_lane_until_junction(self, world, current_wp, remaining):
        if self._locked_lane_id is None:
            return remaining

        wmap = world.get_map()
        adjusted = []
        last_locked_wp = None
        last_route_wp = None
        for wp in remaining:
            same_segment = (
                wp.road_id == self._locked_road_id and
                wp.section_id == self._locked_section_id
            )
            if not same_segment:
                adjusted.append(wp)
                last_route_wp = wp
                continue

            locked_wp = self._project_route_wp_to_locked_lane(wmap, wp)

            if (locked_wp is not None and
                    locked_wp.lane_type == carla.LaneType.Driving and
                    self._same_direction_waypoints(wp, locked_wp) and
                    self._stays_on_selected_route_corridor(wp, locked_wp)):
                adjusted.append(locked_wp)
                last_locked_wp = locked_wp
            else:
                fallback_wp = self._advance_locked_lane_on_route(last_locked_wp, last_route_wp, wp)
                if fallback_wp is not None:
                    adjusted.append(fallback_wp)
                    last_locked_wp = fallback_wp
            last_route_wp = wp
        return adjusted

    def _temporary_lane_until_route_maneuver(self, world, current_wp, remaining):
        if self._manual_lane_id is None:
            return remaining

        wmap = world.get_map()
        adjusted = []
        last_temp_wp = None
        last_route_wp = None
        for wp in remaining:
            if self._route_point_is_close_to_maneuver(wp):
                adjusted.append(wp)
                last_route_wp = wp
                continue

            same_segment = (
                wp.road_id == self._manual_lane_road_id and
                wp.section_id == self._manual_lane_section_id
            )
            if not same_segment:
                adjusted.append(wp)
                last_route_wp = wp
                continue

            temp_wp = self._project_route_wp_to_lane(wmap, wp, self._manual_lane_id)
            if (temp_wp is not None and
                    temp_wp.lane_type == carla.LaneType.Driving and
                    self._same_direction_waypoints(wp, temp_wp) and
                    self._stays_on_temporary_lane_corridor(wp, temp_wp)):
                adjusted.append(temp_wp)
                last_temp_wp = temp_wp
            else:
                fallback_wp = self._advance_lane_on_route(
                    last_temp_wp, last_route_wp, wp, self._manual_lane_id)
                if fallback_wp is not None:
                    adjusted.append(fallback_wp)
                    last_temp_wp = fallback_wp
                else:
                    adjusted.append(wp)
            last_route_wp = wp
        return adjusted

    def _route_point_is_close_to_maneuver(self, wp):
        idx = self._waypoint_index_hint(wp)
        if idx is None:
            return False
        return self._is_close_to_route_maneuver(idx)

    def _is_close_to_route_maneuver(self, idx):
        distance = self._distance_to_route_maneuver(idx)
        return distance is not None and distance <= self.ROUTE_RETURN_DISTANCE_M

    def _project_route_wp_to_locked_lane(self, wmap, route_wp):
        return self._project_route_wp_to_lane(wmap, route_wp, self._locked_lane_id)

    def _project_route_wp_to_lane(self, wmap, route_wp, lane_id):
        try:
            return wmap.get_waypoint_xodr(
                route_wp.road_id,
                lane_id,
                route_wp.s,
            )
        except Exception:
            return None

    def _advance_locked_lane_on_route(self, last_locked_wp, last_route_wp, route_wp):
        return self._advance_lane_on_route(
            last_locked_wp, last_route_wp, route_wp, self._locked_lane_id)

    def _advance_lane_on_route(self, last_locked_wp, last_route_wp, route_wp, lane_id):
        if last_locked_wp is None:
            return None
        if last_route_wp is None:
            step = 2.0
        else:
            step = last_route_wp.transform.location.distance(route_wp.transform.location)
            step = max(1.0, min(step, 4.0))

        try:
            candidates = last_locked_wp.next(step)
        except Exception:
            return None
        if not candidates:
            return None

        valid = []
        for candidate in candidates:
            if candidate.road_id != route_wp.road_id:
                continue
            if candidate.section_id != route_wp.section_id:
                continue
            if candidate.lane_id != lane_id:
                continue
            if candidate.lane_type != carla.LaneType.Driving:
                continue
            if not self._same_direction_waypoints(route_wp, candidate):
                continue
            if not self._stays_on_selected_route_corridor(route_wp, candidate):
                continue
            valid.append(candidate)

        if not valid:
            return None
        return min(valid, key=lambda wp: wp.transform.location.distance(route_wp.transform.location))

    def _waypoint_index_hint(self, route_wp):
        if not hasattr(route_wp, "road_id"):
            return None
        start = max(0, self.wp_idx - 4)
        stop = min(len(self.waypoints), self.wp_idx + 80)
        best_idx = None
        best_score = float("inf")
        for i in range(start, stop):
            wp = self.waypoints[i]
            if wp.road_id != route_wp.road_id or wp.section_id != route_wp.section_id:
                continue
            score = abs(getattr(wp, "s", 0.0) - getattr(route_wp, "s", 0.0))
            if score < best_score:
                best_score = score
                best_idx = i
        return best_idx

    def _smooth_return_to_locked_lane(self, current_wp, remaining):
        if not remaining:
            return remaining

        start = self.vehicle.get_location()
        safe_remaining = self._remaining_before_maneuver(remaining)
        if not safe_remaining:
            return remaining

        target_idx = None
        target_loc = None
        target_forward = None
        for i, point in enumerate(reversed(safe_remaining)):
            loc = _route_point_location(point)
            if loc is None:
                continue
            dist = start.distance(loc)
            if dist < 10.0:
                continue
            target_idx = len(safe_remaining) - 1 - i
            target_loc = loc
            transform = getattr(point, "transform", None)
            if transform is not None:
                target_forward = transform.get_forward_vector()
            break
        if target_idx is None or target_loc is None:
            for i, point in enumerate(reversed(safe_remaining)):
                loc = _route_point_location(point)
                if loc is None:
                    continue
                target_idx = len(safe_remaining) - 1 - i
                target_loc = loc
                transform = getattr(point, "transform", None)
                if transform is not None:
                    target_forward = transform.get_forward_vector()
                break
        if target_idx is None or target_loc is None:
            return remaining

        vehicle_tf = self.vehicle.get_transform()
        start_forward = vehicle_tf.get_forward_vector()
        direct = target_loc - start
        direct_len = max((direct.x ** 2 + direct.y ** 2) ** 0.5, 1.0)
        if target_forward is None:
            target_forward = carla.Vector3D(direct.x / direct_len, direct.y / direct_len, 0.0)

        heading_dot = start_forward.x * target_forward.x + start_forward.y * target_forward.y
        if heading_dot < 0.50:
            return remaining

        handle = min(max(direct_len * 1.10, 16.0), 32.0)
        p0 = start
        p1 = start + carla.Location(
            x=start_forward.x * handle,
            y=start_forward.y * handle,
            z=0.0,
        )
        p2 = target_loc - carla.Location(
            x=target_forward.x * handle,
            y=target_forward.y * handle,
            z=0.0,
        )
        p3 = target_loc

        lead_in = [
            start + carla.Location(
                x=start_forward.x * d,
                y=start_forward.y * d,
                z=0.0,
            )
            for d in (3.0, 6.0, 9.0)
            if d < direct_len * 0.45
        ]
        blend = [self._bezier_location(p0, p1, p2, p3, t)
                 for t in (0.04, 0.09, 0.14, 0.19, 0.24, 0.29, 0.34, 0.39,
                           0.44, 0.49, 0.54, 0.59, 0.64, 0.69, 0.74, 0.79,
                           0.84, 0.89, 0.94, 0.98)]
        return lead_in + blend + remaining[target_idx:]

    def _remaining_before_maneuver(self, remaining):
        maneuver_idx = self._route_maneuver_index(self.wp_idx)
        if maneuver_idx is None:
            return remaining[:16]

        rel_idx = max(0, maneuver_idx - self.wp_idx)
        stop = self._pre_maneuver_stop_index(remaining, rel_idx)
        safe = []
        for point in remaining[:stop]:
            if getattr(point, "is_junction", False):
                break
            safe.append(point)
        return safe

    def _pre_maneuver_stop_index(self, remaining, rel_idx):
        if rel_idx <= 0:
            return 1

        distance_back = 0.0
        stop = max(1, min(len(remaining), rel_idx))
        prev_loc = _route_point_location(remaining[min(rel_idx, len(remaining) - 1)])
        for i in range(min(rel_idx, len(remaining) - 1) - 1, -1, -1):
            loc = _route_point_location(remaining[i])
            if loc is None or prev_loc is None:
                prev_loc = loc
                continue
            distance_back += loc.distance(prev_loc)
            prev_loc = loc
            if distance_back >= self.ROUTE_RETURN_COMPLETE_BEFORE_M:
                stop = i + 1
                break
        return max(1, min(len(remaining), stop))

    def _bezier_location(self, p0, p1, p2, p3, t):
        u = 1.0 - t
        return carla.Location(
            x=(u ** 3) * p0.x + 3 * (u ** 2) * t * p1.x + 3 * u * (t ** 2) * p2.x + (t ** 3) * p3.x,
            y=(u ** 3) * p0.y + 3 * (u ** 2) * t * p1.y + 3 * u * (t ** 2) * p2.y + (t ** 3) * p3.y,
            z=(u ** 3) * p0.z + 3 * (u ** 2) * t * p1.z + 3 * u * (t ** 2) * p2.z + (t ** 3) * p3.z,
        )

    def _path_key(self, route_points):
        key = []
        for point in route_points[:16]:
            if hasattr(point, "road_id"):
                key.append((
                    point.road_id,
                    getattr(point, "section_id", None),
                    getattr(point, "lane_id", None),
                    round(getattr(point, "s", 0.0), 1),
                ))
                continue
            loc = _route_point_location(point)
            if loc is not None:
                key.append(("loc", round(loc.x, 1), round(loc.y, 1)))
        return tuple(key)

    def _stays_on_selected_route_corridor(self, route_wp, candidate_wp):
        if candidate_wp.road_id != route_wp.road_id:
            return False
        if candidate_wp.section_id != route_wp.section_id:
            return False

        route_loc = route_wp.transform.location
        cand_loc = candidate_wp.transform.location
        max_offset = max(getattr(route_wp, "lane_width", 3.5) * 2.25, 8.0)
        return route_loc.distance(cand_loc) <= max_offset

    def _stays_on_temporary_lane_corridor(self, route_wp, candidate_wp):
        if candidate_wp.road_id != route_wp.road_id:
            return False
        if candidate_wp.section_id != route_wp.section_id:
            return False

        route_loc = route_wp.transform.location
        cand_loc = candidate_wp.transform.location
        max_offset = max(getattr(route_wp, "lane_width", 3.5) * 3.5, 13.0)
        return route_loc.distance(cand_loc) <= max_offset

    def _same_direction_waypoints(self, a_wp, b_wp):
        if a_wp is None or b_wp is None:
            return False
        if getattr(a_wp, "lane_id", 0) != 0 and getattr(b_wp, "lane_id", 0) != 0:
            if (a_wp.lane_id > 0) != (b_wp.lane_id > 0):
                return False
        a_forward = a_wp.transform.get_forward_vector()
        b_forward = b_wp.transform.get_forward_vector()
        return (a_forward.x * b_forward.x + a_forward.y * b_forward.y) > 0.55

    def _clear_manual_lane(self):
        self._manual_lane_road_id = None
        self._manual_lane_section_id = None
        self._manual_lane_id = None
        self._accept_lane_after_manual_change = False
        self._manual_lane_change_pending = False
        self._manual_pending_frames = 0

    def _off_selected_route(self, world, loc, idx):
        try:
            current_wp = world.get_map().get_waypoint(
                loc,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
        except Exception:
            return False
        if current_wp is None:
            return False

        nearby = self.waypoints[max(0, idx - 10):min(len(self.waypoints), idx + 24)]
        if not nearby:
            return False

        nearest_route_wp = min(nearby, key=lambda wp: loc.distance(wp.transform.location))
        if loc.distance(nearest_route_wp.transform.location) > 5.0:
            return True

        same_route_road = any(
            wp.road_id == current_wp.road_id and
            (not getattr(current_wp, "is_junction", False) or wp.section_id == current_wp.section_id)
            for wp in nearby
        )
        if same_route_road:
            return False

        return loc.distance(nearest_route_wp.transform.location) > 3.5


def _start_tm_lane_change(traffic_manager, vehicle, direction, world=None):
    target_wp = _safe_lane_change_target(world, vehicle, direction)
    if target_wp is None:
        return None

    vehicle.set_autopilot(False)
    vehicle.set_autopilot(True, traffic_manager.get_port())
    traffic_manager.auto_lane_change(vehicle, False)
    traffic_manager.set_desired_speed(vehicle, 24.0)
    _force_tm_lane_change(traffic_manager, vehicle, direction)
    return {
        "direction": direction,
        "frames": 0,
        "stable_frames": 0,
        "target_road_id": target_wp.road_id,
        "target_section_id": target_wp.section_id,
        "target_lane_id": target_wp.lane_id,
    }


def _update_tm_lane_change(traffic_manager, vehicle, task, world=None):
    if not task:
        return None

    task["frames"] += 1
    _force_tm_lane_change(traffic_manager, vehicle, task["direction"])

    current_wp = _current_driving_waypoint(world, vehicle)
    if current_wp is not None:
        on_target_lane = (
            current_wp.road_id == task["target_road_id"] and
            current_wp.section_id == task["target_section_id"] and
            current_wp.lane_id == task["target_lane_id"]
        )
        if on_target_lane:
            task["stable_frames"] += 1
        else:
            task["stable_frames"] = 0

    if task["stable_frames"] >= 3:
        traffic_manager.set_desired_speed(vehicle, 30.0)
        log(f"Traffic Manager lane change completed: {task['direction']}")
        return None

    if task["frames"] >= 90:
        traffic_manager.set_desired_speed(vehicle, 30.0)
        log(f"Traffic Manager lane change timed out: {task['direction']}", "!")
        return None

    return task


def _force_tm_lane_change(traffic_manager, vehicle, direction):
    traffic_manager.force_lane_change(vehicle, direction == "right")


def _safe_lane_change_target(world, vehicle, direction):
    current_wp = _current_driving_waypoint(world, vehicle)
    if current_wp is None or current_wp.is_junction:
        return None

    direction = direction.lower().strip()
    desired_side = 1 if direction in ("right", "r", "1") else -1
    right = current_wp.transform.get_right_vector()
    forward = current_wp.transform.get_forward_vector()

    best = None
    for adjacent in (current_wp.get_left_lane(), current_wp.get_right_lane()):
        if adjacent is None:
            continue
        if adjacent.lane_type != carla.LaneType.Driving or adjacent.is_junction:
            continue
        if not _same_direction_lane(current_wp, adjacent):
            continue
        if not _lane_marking_allows(current_wp, desired_side):
            continue

        adj_forward = adjacent.transform.get_forward_vector()
        if forward.x * adj_forward.x + forward.y * adj_forward.y < 0.65:
            continue

        delta = adjacent.transform.location - current_wp.transform.location
        side = delta.x * right.x + delta.y * right.y
        if side * desired_side <= 0.2:
            continue

        if not _target_lane_continues(adjacent):
            continue

        candidate = (abs(side), adjacent)
        if best is None or candidate[0] < best[0]:
            best = candidate

    return best[1] if best else None


def _current_driving_waypoint(world, vehicle):
    if world is None:
        try:
            world = vehicle.get_world()
        except Exception:
            return None
    try:
        return world.get_map().get_waypoint(
            vehicle.get_location(),
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
    except Exception:
        return None


def _same_direction_lane(current_wp, adjacent_wp):
    if current_wp.lane_id == 0 or adjacent_wp.lane_id == 0:
        return True
    return (current_wp.lane_id > 0) == (adjacent_wp.lane_id > 0)


def _lane_marking_allows(wp, desired_side):
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


def _target_lane_continues(wp):
    try:
        next_wps = wp.next(18.0)
    except Exception:
        return False
    if not next_wps:
        return False
    return any(
        nxt.lane_type == carla.LaneType.Driving and
        nxt.road_id == wp.road_id and
        nxt.lane_id == wp.lane_id
        for nxt in next_wps
    )


def _apply_weather(world, mode):
    presets = {
        "sunny": carla.WeatherParameters(
            cloudiness=5.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=5.0,
            sun_azimuth_angle=45.0,
            sun_altitude_angle=70.0,
            fog_density=0.0,
            wetness=0.0,
        ),
        "rainy": carla.WeatherParameters(
            cloudiness=85.0,
            precipitation=85.0,
            precipitation_deposits=75.0,
            wind_intensity=45.0,
            sun_azimuth_angle=45.0,
            sun_altitude_angle=25.0,
            fog_density=15.0,
            wetness=85.0,
        ),
        "snowy": carla.WeatherParameters(
            cloudiness=95.0,
            precipitation=20.0,
            precipitation_deposits=90.0,
            wind_intensity=35.0,
            sun_azimuth_angle=45.0,
            sun_altitude_angle=18.0,
            fog_density=45.0,
            fog_distance=25.0,
            wetness=100.0,
        ),
    }

    weather = presets.get(mode)
    if weather is None:
        return

    world.set_weather(weather)
    log(f"Weather changed: {mode}")


def _draw_carla_lane_debug(world, vehicle, target_wp=None, route_points=None):
    """Draw lane-following guide lines directly inside the CARLA viewport."""
    try:
        wmap = world.get_map()
        wp = target_wp or wmap.get_waypoint(
            vehicle.get_location(),
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if wp is None:
            return

        seed_wp = wp
        if route_points and target_wp is None:
            for point in route_points[:6]:
                candidate = point if hasattr(point, "transform") else None
                loc = _route_point_location(point)
                if candidate is None and loc is not None:
                    candidate = wmap.get_waypoint(
                        loc,
                        project_to_road=True,
                        lane_type=carla.LaneType.Driving,
                    )
                if candidate is None or candidate.lane_type != carla.LaneType.Driving:
                    continue
                if vehicle.get_location().distance(candidate.transform.location) < 18.0:
                    seed_wp = candidate
                    break

        waypoints = _lane_center_debug_waypoints(seed_wp)
        if len(waypoints) < 2:
            return

        life = 0.07
        z_lift = carla.Location(z=0.08)
        for a, b in zip(waypoints, waypoints[1:]):
            ar = a.transform.get_right_vector()
            br = b.transform.get_right_vector()
            half_a = max(a.lane_width * 0.5, 1.5)
            half_b = max(b.lane_width * 0.5, 1.5)

            a_center = a.transform.location + z_lift
            b_center = b.transform.location + z_lift
            a_left = a_center - carla.Location(x=ar.x * half_a, y=ar.y * half_a)
            b_left = b_center - carla.Location(x=br.x * half_b, y=br.y * half_b)
            a_right = a_center + carla.Location(x=ar.x * half_a, y=ar.y * half_a)
            b_right = b_center + carla.Location(x=br.x * half_b, y=br.y * half_b)

            world.debug.draw_line(a_left, b_left, thickness=0.12,
                                  color=carla.Color(0, 255, 255),
                                  life_time=life, persistent_lines=False)
            world.debug.draw_line(a_right, b_right, thickness=0.12,
                                  color=carla.Color(255, 0, 255),
                                  life_time=life, persistent_lines=False)
            world.debug.draw_line(a_center, b_center, thickness=0.08,
                                  color=carla.Color(255, 220, 0),
                                  life_time=life, persistent_lines=False)
    except Exception:
        pass


def _lane_center_debug_waypoints(seed_wp, steps=22, step_m=2.5):
    """Build a contiguous CARLA lane-center chain so debug lines never cut across lanes."""
    waypoints = [seed_wp]
    current = seed_wp
    for _ in range(steps):
        try:
            candidates = current.next(step_m)
        except Exception:
            break
        if not candidates:
            break

        same_lane = [
            wp for wp in candidates
            if wp.lane_type == carla.LaneType.Driving and
            wp.road_id == current.road_id and
            wp.section_id == current.section_id and
            wp.lane_id == current.lane_id
        ]
        drivable = [
            wp for wp in candidates
            if wp.lane_type == carla.LaneType.Driving
        ]
        next_wp = same_lane[0] if same_lane else (drivable[0] if drivable else None)
        if next_wp is None:
            break
        waypoints.append(next_wp)
        current = next_wp
    return waypoints
