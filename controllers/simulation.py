"""
controllers/simulation.py — Main simulation loop (orchestration)
"""

import time
import math
import carla

from config import TARGET_SPEED_KMH, GOAL_M, MAX_S, FIXED_DELTA
from utils.logger import sec
from views.spectator import spectator_update
from views.green_line import green_line
from controllers.vehicle_controller import compute_control


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
#  LANE FOLLOWING SIMULATION LOOP
# =====================================================================

def run_lane(world, vehicle, waypoints, wmap, end_loc,
             lane_ctrl, lane_cam, minimap=None):
    """
    Lane following simulation loop.

    Ana fark: Direksiyon kontrolü kamera görüntüsünden şerit tespiti ile yapılır.
    Şerit kaybolursa (kavşak, gölge vb.) waypoint PID'ye fallback yapar.

    Args:
        lane_ctrl : LaneController instance
        lane_cam  : LaneCam instance (ön kamera sensörü)
        dcam      : DroneCam (opsiyonel, Pygame drone penceresi)
        minimap   : MiniMap (opsiyonel)
    """
    from config import LANE_SPEED_KMH, LANE_LOST_MAX

    sec("6 – Simulation Loop  (LANE FOLLOWING + PID Fallback)")
    frame    = 0
    t0       = time.time()
    wp_idx   = 0
    stall_t  = 0
    lost_cnt = 0       # Şerit kaybı sayacı
    fb_cnt   = 0       # Fallback (waypoint) frame sayısı

    # Spectator'ı başlangıçta araca taşı (sonrası serbest kontrol)
    spec = world.get_spectator()
    spectator_update(spec, vehicle)

    print(f"  Waypoints  : {len(waypoints)}")
    print(f"  Goal       : ({end_loc.x:.0f}, {end_loc.y:.0f})")
    print(f"  Target spd : {LANE_SPEED_KMH} km/h  (lane mode)")
    print(f"  Fallback   : Waypoint PID after {LANE_LOST_MAX} lost frames")
    print()
    print(f"  {'F':>6}  {'t':>5}  {'km/h':>6}  {'dist':>7}  {'mode':>8}  thr   br   st")
    print(f"  {'─'*6}  {'─'*5}  {'─'*6}  {'─'*7}  {'─'*8}  {'─'*5} {'─'*4} {'─'*5}")

    while True:
        tick_start = time.perf_counter()

        # ── Kamera frame'i al ─────────────────────────────────────────
        cam_frame = lane_cam.get_frame()

        # ── Hız bilgisi ──────────────────────────────────────────────
        try:
            vel = vehicle.get_velocity()
            spd_ms = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            spd = spd_ms * 3.6
        except RuntimeError:
            print("\n  [!] Vehicle unreachable")
            return {"ok": False, "f": frame, "t": time.time() - t0}

        # ── Şerit takip kontrolü ─────────────────────────────────────
        mode = "LANE"
        if cam_frame is not None:
            ctrl, lane_ok = lane_ctrl.process_frame(cam_frame, spd_ms)

            if lane_ok:
                lost_cnt = 0
                fb_cnt = 0
            else:
                lost_cnt += 1

            # Şerit çok uzun süredir kayıp → waypoint fallback
            if lost_cnt >= LANE_LOST_MAX:
                ctrl, wp_idx = compute_control(vehicle, waypoints, wp_idx)
                mode = "WP-FB"
                fb_cnt += 1

                # Fallback'ten çıkış: şerit tekrar bulunursa
                if lane_ok:
                    lost_cnt = 0
                    fb_cnt = 0
        else:
            # Kamera henüz hazır değil → waypoint ile başla
            ctrl, wp_idx = compute_control(vehicle, waypoints, wp_idx)
            mode = "WP-INIT"

        # ── Waypoint index güncelle (lane modunda da takip için) ──────
        # Lane modunda bile wp_idx'i güncel tut (mesafe hesabı + fallback)
        try:
            loc = vehicle.get_location()
            while wp_idx < len(waypoints) - 1:
                wp_loc = waypoints[wp_idx].transform.location
                if loc.distance(wp_loc) > 8.0:
                    break
                wp_idx += 1
        except RuntimeError:
            pass

        # ── Kontrolü uygula ──────────────────────────────────────────
        vehicle.apply_control(ctrl)

        # ── Simulation step ──────────────────────────────────────────
        world.tick()
        frame += 1
        elapsed = time.time() - t0

        # ── Green line + kamera takibi ────────────────────────────────
        green_line(world, vehicle, waypoints, wp_idx)
        spectator_update(spec, vehicle)

        # ── Mesafe hesabı ────────────────────────────────────────────
        try:
            loc = vehicle.get_location()
            dist = loc.distance(waypoints[wp_idx].transform.location)
            for i in range(wp_idx, len(waypoints) - 1):
                dist += waypoints[i].transform.location.distance(
                    waypoints[i + 1].transform.location
                )
        except RuntimeError:
            print("\n  [!] Vehicle unreachable")
            return {"ok": False, "f": frame, "t": elapsed}

        # ── Lane debug penceresi ─────────────────────────────────────
        if not lane_ctrl.show_debug():
            print("\n  [!] Lane debug window closed (ESC)")
            return {"ok": False, "f": frame, "t": elapsed}

        # ── MiniMap ──────────────────────────────────────────────────
        if minimap:
            minimap.render(vehicle.get_transform(), waypoints, wp_idx)

        # ── Log (every 40 frames) ────────────────────────────────────
        if frame % 40 == 0:
            print(f"  {frame:>6}  {elapsed:>4.1f}s  "
                  f"{spd:>5.1f}  {dist:>6.0f}m  "
                  f"{mode:>8}  "
                  f"{ctrl.throttle:.2f}  {ctrl.brake:.2f}  {ctrl.steer:+.2f}")

        # ── Route completed ──────────────────────────────────────────
        if wp_idx >= len(waypoints) - 1:
            stop = carla.VehicleControl(throttle=0.0, brake=1.0)
            for _ in range(40):
                vehicle.apply_control(stop)
                world.tick()
            print(f"\n  {'═'*55}")
            print(f"  [🏁] ROUTE COMPLETED!  {elapsed:.1f}s  {frame} frames")
            print(f"  Lane frames: {frame - fb_cnt}  Fallback frames: {fb_cnt}")
            print(f"  {'═'*55}")
            _wait_before_close(world, vehicle, minimap, waypoints, wp_idx)
            if minimap:
                minimap.destroy()
            lane_ctrl.destroy()
            return {"ok": True, "f": frame, "t": elapsed}

        # ── Stall detector ───────────────────────────────────────────
        if spd < 0.5:
            stall_t += 1
            if stall_t >= 100:
                print(f"\n  [!] {stall_t//20}s stalled! RAW THROTTLE override...")
                oc = carla.VehicleControl(throttle=0.8, steer=0.0,
                                          brake=0.0, hand_brake=False)
                for _ in range(40):
                    vehicle.apply_control(oc)
                    world.tick()
                stall_t = 0
                print("  [~] Override done, back to lane following.")
        else:
            stall_t = 0

        # ── Goal distance ────────────────────────────────────────────
        if dist < GOAL_M:
            stop = carla.VehicleControl(throttle=0.0, brake=1.0)
            for _ in range(40):
                vehicle.apply_control(stop)
                world.tick()
            print(f"\n  {'═'*55}")
            print(f"  [🏁] GOAL REACHED!  {elapsed:.1f}s  {frame} frames")
            print(f"  Lane frames: {frame - fb_cnt}  Fallback frames: {fb_cnt}")
            print(f"  {'═'*55}")
            _wait_before_close(world, vehicle, minimap, waypoints, wp_idx)
            if minimap:
                minimap.destroy()
            lane_ctrl.destroy()
            return {"ok": True, "f": frame, "t": elapsed}

        # ── Timeout ──────────────────────────────────────────────────
        if elapsed > MAX_S:
            print(f"\n  [!] Timeout ({MAX_S}s)")
            lane_ctrl.destroy()
            return {"ok": False, "f": frame, "t": elapsed}

        # ── Real-time sync (gerçek zamana senkronizasyon) ────────────
        tick_elapsed = time.perf_counter() - tick_start
        sleep_time = FIXED_DELTA - tick_elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    lane_ctrl.destroy()
    return {"ok": False, "f": frame, "t": time.time() - t0}

