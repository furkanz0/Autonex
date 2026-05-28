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

    def __init__(self):
        self._last_pressed = False
        self._last_command = None
        self._last_weather_pressed = False

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

    @staticmethod
    def _down(vk):
        state = ctypes.windll.user32.GetAsyncKeyState(vk)
        return bool(state & 0x8000 or state & 0x0001)


def run_lane(world, vehicle, end_loc=None, initial_lane_change=None, client=None):
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

    # Wait for first camera frame
    log("Waiting for camera frame...")
    for _ in range(10):
        world.tick()
    if cam.frame is None:
        log("No camera frame received!", "!")

    # ── Initial throttle burst to get the vehicle moving ──────────
    log("Gentle initial roll...")
    kick = carla.VehicleControl(throttle=0.35, steer=0.0, brake=0.0)
    for _ in range(8):
        vehicle.apply_control(kick)
        world.tick()

    if initial_lane_change and controller.request_lane_change(initial_lane_change):
        log(f"Initial lane change requested: {initial_lane_change}")

    frame = 0
    t0 = time.time()
    stall_t = 0

    goal_str = f"({end_loc.x:.0f}, {end_loc.y:.0f})" if end_loc else "None (free drive)"
    print(f"  Mode       : PURE VISION (OpenCV lane + traffic light)")
    print(f"  Goal       : {goal_str}")
    print(f"  Control    : Camera PID (no CARLA waypoints)")
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

        # ── Compute control ──────────────────────────────────────────
        if tl_result.should_stop:
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
            vehicle.apply_control(ctrl)
        else:
            # Normal sürüş — kamera PID
            ctrl = controller.compute(lane_result, spd)
            vehicle.apply_control(ctrl)

        # ── Tick ─────────────────────────────────────────────────────
        world.tick()
        frame += 1
        elapsed = time.time() - t0

        # ── Spectator ────────────────────────────────────────────────
        spectator_update(spec, vehicle)
        _draw_carla_lane_debug(world, vehicle, controller.debug_lane_waypoint)

        # ── Dashboard ────────────────────────────────────────────────
        if not dashboard.render(
                lane_result, spd, ctrl.steer, frame,
                controller.lane_change_state, controller.target_offset_m,
                tl_state=tl_result.state,
                tl_confirmed=tl_result.confirmed):
            print("\n  [!] Window closed")
            _cleanup_lane(cam, dashboard)
            return {"ok": False, "f": frame, "t": elapsed}

        command = dashboard.consume_command() or keyboard.poll()
        if command and controller.request_lane_change(command):
            log(f"Lane change requested: {command}")

        weather_command = dashboard.consume_weather() or keyboard.poll_weather()
        if weather_command:
            _apply_weather(world, weather_command)

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
            if dist < GOAL_M:
                stop = carla.VehicleControl(throttle=0.0, brake=1.0)
                for _ in range(40):
                    vehicle.apply_control(stop)
                    world.tick()
                print(f"\n  {'═'*55}")
                print(f"  [🏁] GOAL REACHED!  {elapsed:.1f}s  {frame} frames")
                print(f"  {'═'*55}")
                _cleanup_lane(cam, dashboard)
                return {"ok": True, "f": frame, "t": elapsed}

        # ── Stall detector ───────────────────────────────────────────
        if tl_result.should_stop:
            stall_t = 0  # kırmızı ışıkta durma stall değil
        elif spd < 0.5:
            stall_t += 1
            if stall_t >= 40:
                print(f"\n  [!] {stall_t//20}s stalled! low-speed assist (1s)...")
                oc = carla.VehicleControl(
                    throttle=0.45,
                    steer=float(controller.last_pid.get("steer", 0.0)),
                    brake=0.0, hand_brake=False)
                for _ in range(20):
                    vehicle.apply_control(oc)
                    world.tick()
                stall_t = 0
                print("  [~] Assist done, back to vision PID.")
        else:
            stall_t = 0

        # ── Timeout ──────────────────────────────────────────────────
        if elapsed > MAX_S:
            print(f"\n  [!] Timeout ({MAX_S}s)")
            _cleanup_lane(cam, dashboard)
            return {"ok": False, "f": frame, "t": elapsed}

    _cleanup_lane(cam, dashboard)
    return {"ok": False, "f": frame, "t": time.time() - t0}


# =====================================================================
#  LANE MODE YARDIMCI FONKSİYONLAR
# =====================================================================

def _cleanup_lane(cam, dashboard):
    """Destroy camera and dashboard."""
    try:
        cam.destroy()
    except Exception:
        pass
    try:
        dashboard.destroy()
    except Exception:
        pass





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


def _draw_carla_lane_debug(world, vehicle, target_wp=None):
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

        waypoints = [wp]
        current = wp
        for _ in range(18):
            nxt = current.next(2.5)
            if not nxt:
                break
            current = nxt[0]
            waypoints.append(current)

        life = 0.11
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
                                  color=carla.Color(0, 255, 255), life_time=life)
            world.debug.draw_line(a_right, b_right, thickness=0.12,
                                  color=carla.Color(255, 0, 255), life_time=life)
            world.debug.draw_line(a_center, b_center, thickness=0.08,
                                  color=carla.Color(255, 220, 0), life_time=life)
    except Exception:
        pass

