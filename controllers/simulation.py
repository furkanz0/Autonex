"""
controllers/simulation.py — Main simulation loop (orchestration)
"""

import time
import math
import carla

from config import TARGET_SPEED_KMH, GOAL_M, MAX_S
from utils.logger import sec
from views.spectator import spectator_update
from views.green_line import green_line
from controllers.vehicle_controller import compute_control


def run(world, vehicle, waypoints, wmap, end_loc, dcam=None):
    """
    Main simulation loop.
    Compute control → tick → update spectator/line → telemetry → goal/time check
    """
    sec("6 – Simulation Loop  (Proportional Waypoint Control)")
    spec     = world.get_spectator()
    frame    = 0
    t0       = time.time()
    wp_idx   = 0
    stall_t  = 0   # zero-speed counter

    print(f"  Waypoints  : {len(waypoints)}")
    print(f"  Goal       : ({end_loc.x:.0f}, {end_loc.y:.0f})")
    print(f"  Target spd : {TARGET_SPEED_KMH} km/h")
    print()
    print(f"  {'F':>6}  {'t':>5}  {'km/h':>6}  {'dist':>7}  thr   br   st")
    print(f"  {'─'*6}  {'─'*5}  {'─'*6}  {'─'*7}  {'─'*5} {'─'*4} {'─'*5}")

    while True:
        # ── Compute control ──────────────────────────────────────────
        ctrl, wp_idx = compute_control(vehicle, waypoints, wp_idx)
        vehicle.apply_control(ctrl)

        # ── Simulation step ──────────────────────────────────────────
        world.tick()
        frame += 1
        elapsed = time.time() - t0

        # ── Spectator + green line ───────────────────────────────────
        spectator_update(spec, vehicle)
        green_line(world, wmap, vehicle)

        # ── Telemetry ────────────────────────────────────────────────
        try:
            vel  = vehicle.get_velocity()
            spd  = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            loc  = vehicle.get_location()
            dist = loc.distance(end_loc)
        except RuntimeError:
            print("\n  [!] Vehicle unreachable")
            return {"ok": False, "f": frame, "t": elapsed}

        # ── Pygame ───────────────────────────────────────────────────
        if dcam and not dcam.render(spd, dist):
            print("\n  [!] Window closed")
            return {"ok": False, "f": frame, "t": elapsed}

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
            _wait_before_close(dcam, world, vehicle)
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
            _wait_before_close(dcam, world, vehicle)
            return {"ok": True, "f": frame, "t": elapsed}

        # ── Timeout ──────────────────────────────────────────────────
        if elapsed > MAX_S:
            print(f"\n  [!] Timeout ({MAX_S}s)")
            return {"ok": False, "f": frame, "t": elapsed}

    return {"ok": False, "f": frame, "t": time.time() - t0}


def _wait_before_close(dcam, world, vehicle):
    """Keep simulation visible for 5 seconds after goal reached."""
    print("  [~] Waiting 5s before cleanup...")
    for _ in range(100):  # 5s = 100 × 0.05
        world.tick()
        try:
            vel = vehicle.get_velocity()
            spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            loc = vehicle.get_location()
            if dcam and not dcam.render(spd, 0):
                break
        except Exception:
            break
