"""
controllers/lane_simulation.py — Simulation loop for lane-following mode

Replaces the waypoint-based simulation loop with a vision-based one.
Uses LaneCamera → LaneDetector → LaneController pipeline.
"""

import time
import math
import carla

from config import MAX_S, GOAL_M
from utils.logger import sec, log
from views.spectator import spectator_update
from views.lane_camera import LaneCamera
from views.lane_dashboard import LaneDashboard
from models.lane_detector import LaneDetector
from controllers.lane_controller import LaneController


def run_lane(world, vehicle, end_loc=None):
    """
    Lane-following simulation loop.

    The vehicle drives by detecting lane markings with OpenCV,
    not by following pre-computed waypoints.

    Args:
        world: CARLA world
        vehicle: spawned vehicle actor
        end_loc: optional goal location (if None, drives indefinitely)

    Returns:
        dict with ok, f (frames), t (time)
    """
    sec("6 – Simulation Loop  (Lane Detection Mode)")

    # ── Initialize components ────────────────────────────────────────
    cam = LaneCamera(world, vehicle)
    detector = LaneDetector()
    controller = LaneController()
    dashboard = LaneDashboard()
    spec = world.get_spectator()

    # Wait for first camera frame
    log("Waiting for camera frame...")
    for _ in range(10):
        world.tick()
    if cam.frame is None:
        log("No camera frame received!", "!")

    # ── Initial throttle burst to get the vehicle moving ──────────
    log("Initial throttle burst (1s)...")
    kick = carla.VehicleControl(throttle=0.7, steer=0.0, brake=0.0)
    for _ in range(20):
        vehicle.apply_control(kick)
        world.tick()

    frame = 0
    t0 = time.time()
    stall_t = 0

    goal_str = f"({end_loc.x:.0f}, {end_loc.y:.0f})" if end_loc else "None (free drive)"
    print(f"  Mode       : LANE DETECTION (OpenCV)")
    print(f"  Goal       : {goal_str}")
    print()
    print(f"  {'F':>6}  {'t':>5}  {'km/h':>6}  {'off':>6}  {'curv':>7}  {'conf':>5}  st")
    print(f"  {'─'*6}  {'─'*5}  {'─'*6}  {'─'*6}  {'─'*7}  {'─'*5}  {'─'*6}")

    while True:
        # ── Get camera frame ─────────────────────────────────────────
        raw = cam.frame

        # ── Detect lanes ─────────────────────────────────────────────
        lane_result = detector.process(raw)

        # ── Compute speed ────────────────────────────────────────────
        vel = vehicle.get_velocity()
        spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

        # ── Compute control ──────────────────────────────────────────
        ctrl = controller.compute(lane_result, spd)
        vehicle.apply_control(ctrl)

        # ── Tick ─────────────────────────────────────────────────────
        world.tick()
        frame += 1
        elapsed = time.time() - t0

        # ── Spectator ────────────────────────────────────────────────
        spectator_update(spec, vehicle)

        # ── Dashboard ────────────────────────────────────────────────
        if not dashboard.render(lane_result, spd, ctrl.steer, frame):
            print("\n  [!] Window closed")
            _cleanup(cam, dashboard)
            return {"ok": False, "f": frame, "t": elapsed}

        # ── Console log (every 20 frames) ────────────────────────────
        if frame % 20 == 0:
            off = lane_result.lateral_offset_m
            curv = lane_result.curvature_m
            conf = lane_result.confidence * 100
            det = "✓" if lane_result.detected else "✗"
            print(f"  {frame:>6}  {elapsed:>4.1f}s  "
                  f"{spd:>5.1f}  {off:>+5.2f}m  "
                  f"{curv:>6.0f}m  {conf:>4.0f}%  {ctrl.steer:+.3f}  {det}")

        # ── Goal check ───────────────────────────────────────────────
        if end_loc:
            loc = vehicle.get_location()
            dist = loc.distance(end_loc)
            if dist < GOAL_M:
                # Brake to stop
                stop = carla.VehicleControl(throttle=0.0, brake=1.0)
                for _ in range(40):
                    vehicle.apply_control(stop)
                    world.tick()
                print(f"\n  {'═'*55}")
                print(f"  [🏁] GOAL REACHED!  {elapsed:.1f}s  {frame} frames")
                print(f"  {'═'*55}")
                _cleanup(cam, dashboard)
                return {"ok": True, "f": frame, "t": elapsed}

        # ── Stall detector ───────────────────────────────────────────
        if spd < 0.5:
            stall_t += 1
            if stall_t >= 40:
                print(f"\n  [!] {stall_t//20}s stalled! RAW THROTTLE override (2s)...")
                oc = carla.VehicleControl(throttle=0.8, steer=0.0,
                                          brake=0.0, hand_brake=False)
                for _ in range(40):
                    vehicle.apply_control(oc)
                    world.tick()
                stall_t = 0
                print("  [~] Override done, back to normal.")
        else:
            stall_t = 0

        # ── Timeout ──────────────────────────────────────────────────
        if elapsed > MAX_S:
            print(f"\n  [!] Timeout ({MAX_S}s)")
            _cleanup(cam, dashboard)
            return {"ok": False, "f": frame, "t": elapsed}

    _cleanup(cam, dashboard)
    return {"ok": False, "f": frame, "t": time.time() - t0}


def _cleanup(cam, dashboard):
    """Destroy camera and dashboard."""
    try:
        cam.destroy()
    except Exception:
        pass
    try:
        dashboard.destroy()
    except Exception:
        pass
