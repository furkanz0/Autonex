"""
╔══════════════════════════════════════════════════════════════════════╗
║  main.py — CARLA Town04 Tesla Simulation (MVC Entry Point)          ║
║                                                                      ║
║  Usage:                                                              ║
║    python main.py          → Waypoint control (default route)        ║
║    python main.py --map    → Map navigator + waypoint control        ║
║    python main.py --lane   → OpenCV lane detection driving           ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import sys
import time
import traceback

import carla

from config import WANT_START, WANT_END
from utils.logger import log, sec

from models.connection import connect, load_town04, sync_on, sync_off
from models.vehicle import spawn_tesla, settle_physics, motion_test
from models.route import pick_spawn, snap_end, build_route

from views.drone_cam import DroneCam
from views.map_navigator import MapNavigator

from controllers.simulation import run
from controllers.lane_simulation import run_lane


def run_with_map(client, world, wmap, orig):
    """Interactive route selection via map navigator and test."""
    sec("MAP NAVIGATOR MODE")
    nav = MapNavigator(world)
    result = nav.run()

    if result is None:
        print("  [!] Exited map navigator.")
        return

    start = result["start"]
    end = result["end"]

    log(f"Selected Start: ({start.x:.1f}, {start.y:.1f})")
    log(f"Selected End:   ({end.x:.1f}, {end.y:.1f})")

    # Spawn & vehicle
    spawn_tf = pick_spawn(wmap, start, end)
    vehicle = spawn_tesla(world, spawn_tf)
    settle_physics(world, ticks=15)
    time.sleep(0.2)
    motion_test(world, vehicle)

    # Always compute route from vehicle's ACTUAL position to end
    end_snapped = snap_end(wmap, end)
    waypoints = build_route(wmap, vehicle.get_location(), end_snapped, world)

    if len(waypoints) < 2:
        print("[ERROR] Could not compute route!")
        vehicle.destroy()
        return

    end_loc = waypoints[-1].transform.location
    start_loc = vehicle.get_location()

    # Drone camera & simulation
    dcam = DroneCam(world, vehicle, end_loc, start_loc)
    for _ in range(3):
        world.tick()

    sim_result = run(world, vehicle, waypoints, wmap, end_loc, dcam)

    sec("RESULT")
    print(f"  Status : {'SUCCESS ✓' if sim_result['ok'] else 'INCOMPLETE ✗'}")
    print(f"  Time   : {sim_result['t']:.1f}s")
    print(f"  Frames : {sim_result['f']}")

    dcam.destroy()
    try:
        if vehicle.is_alive:
            vehicle.set_autopilot(False)
            vehicle.destroy()
            log("Vehicle destroyed.")
    except Exception:
        pass


def run_default(client, world, wmap, orig):
    """Test with default (hardcoded) route."""
    vehicle = None
    dcam = None

    try:
        # ── Spawn & target ───────────────────────────────────────────
        spawn_tf = pick_spawn(wmap, WANT_START, WANT_END)
        end_loc  = snap_end(wmap, WANT_END)

        # ── Vehicle ──────────────────────────────────────────────────
        vehicle = spawn_tesla(world, spawn_tf)

        # ── Physics settling ─────────────────────────────────────────
        settle_physics(world, ticks=15)
        time.sleep(0.2)

        # ── Motion diagnostics ───────────────────────────────────────
        motion_test(world, vehicle)

        # ── Route ────────────────────────────────────────────────────
        start_loc = vehicle.get_location()
        waypoints = build_route(wmap, start_loc, end_loc, world)

        if len(waypoints) < 2:
            print("[ERROR] Could not compute route!")
            sys.exit(1)

        # ── Drone camera ─────────────────────────────────────────────
        sec("6a – Drone Camera")
        dcam = DroneCam(world, vehicle, end_loc)
        for _ in range(3):
            world.tick()

        # ── Main loop ────────────────────────────────────────────────
        result = run(world, vehicle, waypoints, wmap, end_loc, dcam)

        # ── Result ───────────────────────────────────────────────────
        sec("RESULT")
        print(f"  Status : {'SUCCESS ✓' if result['ok'] else 'INCOMPLETE ✗'}")
        print(f"  Time   : {result['t']:.1f}s")
        print(f"  Frames : {result['f']}")

    finally:
        if dcam:
            dcam.destroy()
        if vehicle:
            try:
                if vehicle.is_alive:
                    vehicle.set_autopilot(False)
                    vehicle.destroy()
                    log("Vehicle destroyed.")
            except Exception:
                pass


def run_lane_mode(client, world, wmap, orig):
    """OpenCV lane detection driving mode."""
    sec("LANE DETECTION MODE")

    # Spawn vehicle (use default start for now, or --map first)
    spawn_tf = pick_spawn(wmap, WANT_START, WANT_END)
    end_loc = snap_end(wmap, WANT_END)

    vehicle = spawn_tesla(world, spawn_tf)
    settle_physics(world, ticks=15)
    time.sleep(0.2)
    motion_test(world, vehicle)

    sim_result = run_lane(world, vehicle, end_loc)

    sec("RESULT")
    print(f"  Status : {'SUCCESS ✓' if sim_result['ok'] else 'INCOMPLETE ✗'}")
    print(f"  Time   : {sim_result['t']:.1f}s")
    print(f"  Frames : {sim_result['f']}")

    try:
        if vehicle.is_alive:
            vehicle.set_autopilot(False)
            vehicle.destroy()
            log("Vehicle destroyed.")
    except Exception:
        pass


def main():
    client = None
    orig = None
    use_map = "--map" in sys.argv
    use_lane = "--lane" in sys.argv

    if use_lane:
        mode_name = "LANE DETECTION (OpenCV)"
    elif use_map:
        mode_name = "MAP NAVIGATOR"
    else:
        mode_name = "DEFAULT ROUTE (Waypoint)"

    print(f"""
╔══════════════════════════════════════════════════════════════════════╗
║  Town04 Tesla — Autonex Autonomous Driving                         ║
║  Mode: {mode_name:<59s}║
╚══════════════════════════════════════════════════════════════════════╝""")

    try:
        # ── Connection & map ─────────────────────────────────────────
        client = connect()
        world  = load_town04(client)
        wmap   = world.get_map()
        orig   = sync_on(world)

# ======================================================================
# --- CARLA HAVA DURUMU (WEATHER PRESETS) KÜTÜPHANESİ ---
# Not: Sadece kullanmak istediğiniz havanın başındaki '#' işaretini silin.
# ======================================================================

# --- ☀️ GÜNDÜZ (NOON) SENARYOLARI ---
# world.set_weather(carla.WeatherParameters.ClearNoon)       # Pırıl pırıl güneşli, temiz hava
# world.set_weather(carla.WeatherParameters.CloudyNoon)      # Bulutlu ve gölgesiz (Şerit tespiti için en steril hava)
# world.set_weather(carla.WeatherParameters.WetNoon)         # Yağmur yok ama asfalt sırılsıklam ve yansımalı
# world.set_weather(carla.WeatherParameters.HardRainNoon)    # Sağanak yağışlı ve ıslak
# world.set_weather(carla.WeatherParameters.SoftRainNoon)    # Hafif çiseleyen yağmur

# --- 🌅 GÜN BATIMI (SUNSET) SENARYOLARI ---
# world.set_weather(carla.WeatherParameters.ClearSunset)     # Kızıl gün batımı (Kameraya doğrudan güneş vurur)
# world.set_weather(carla.WeatherParameters.WetSunset)       # Gün batımı + Islak asfalt (Göz alan yansımalar)
# world.set_weather(carla.WeatherParameters.HardRainSunset)  # Fırtınalı, karanlık ve kaotik gün batımı

# --- 🌃 GECE (NIGHT) SENARYOLARI ---
# world.set_weather(carla.WeatherParameters.ClearNight)      # Yıldızlı ve bulutsuz zifiri karanlık
# world.set_weather(carla.WeatherParameters.HardRainNight)   # Zifiri karanlıkta sağanak yağış

        world.set_weather(carla.WeatherParameters.ClearNoon)
        log("Weather: ClearNoon")

        if use_lane:
            run_lane_mode(client, world, wmap, orig)
        elif use_map:
            run_with_map(client, world, wmap, orig)
        else:
            run_default(client, world, wmap, orig)

    except KeyboardInterrupt:
        print("\n\n  [!] Ctrl+C")
    except Exception:
        print("\n[CRITICAL ERROR]")
        traceback.print_exc()
    finally:
        sec("Cleanup")
        if orig and client:
            try:
                sync_off(client.get_world(), orig)
            except Exception:
                pass
        print("\n  Done.\n")


if __name__ == "__main__":
    main()
