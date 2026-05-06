"""
models/vehicle.py — Tesla Model3 spawn, physics settling and motion diagnostics
"""

import sys
import math
import carla

from utils.logger import log, sec
from config import TARGET_SPEED_KMH


def spawn_tesla(world, tf):
    """Spawn a red Tesla Model3. Try alternatives on failure."""
    sec("4 – Tesla Model3")
    lib = world.get_blueprint_library()
    bp  = lib.find("vehicle.tesla.model3")
    if bp is None:
        print("[ERROR] tesla.model3 not found")
        sys.exit(1)
    if bp.has_attribute("color"):
        bp.set_attribute("color", "255,0,0")

    v = world.try_spawn_actor(bp, tf)
    if v is None:
        print("  [!] Collision at selected spawn, trying alternatives...")
        for sp in sorted(world.get_map().get_spawn_points(),
                         key=lambda s: s.location.distance(tf.location))[1:20]:
            v = world.try_spawn_actor(bp, sp)
            if v:
                log(f"Alternative spawn: ({sp.location.x:.1f}, {sp.location.y:.1f})")
                break

    if v is None:
        print("[ERROR] Failed to spawn Tesla!")
        sys.exit(1)
    log(f"Tesla ID={v.id}  Red")
    return v


def settle_physics(world, ticks=15):
    """Wait a number of ticks for the vehicle to settle on the ground."""
    log(f"Physics settling ({ticks} ticks)...", "~")
    for _ in range(ticks):
        world.tick()


def motion_test(world, vehicle):
    """
    Apply raw throttle for 20 ticks and check if the vehicle moves.
    If not, wait extra ticks.
    """
    sec("5a – Motion Diagnostics (first 20 ticks raw throttle)")
    print("  Applying raw throttle=0.7, measuring speed...")
    test_ctrl = carla.VehicleControl(throttle=0.7, steer=0.0,
                                      brake=0.0, hand_brake=False)
    max_spd = 0.0
    for i in range(20):
        vehicle.apply_control(test_ctrl)
        world.tick()
        vel = vehicle.get_velocity()
        spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        max_spd = max(max_spd, spd)
        if i % 5 == 0:
            print(f"    tick {i:2d}: speed = {spd:.2f} km/h")

    if max_spd < 0.5:
        print("\n  [WARNING] Speed = 0 despite raw throttle!")
        print("  Possible causes:")
        print("    • Vehicle embedded in ground (bad spawn point)")
        print("    • Sync mode tick not working properly")
        print("    • CARLA server physics engine issue")
        print("  Waiting 15 more ticks and retrying...")
        for _ in range(15):
            world.tick()
    else:
        print(f"\n  [✓] Vehicle is MOVING (max {max_spd:.1f} km/h) – "
              f"switching to control system")

    # Stop and prepare for route start
    stop_ctrl = carla.VehicleControl(throttle=0.0, brake=1.0)
    for _ in range(10):
        vehicle.apply_control(stop_ctrl)
        world.tick()
