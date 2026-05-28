"""
╔══════════════════════════════════════════════════════════════════════╗
║  main.py — CARLA Town10HD Tesla Simulation (MVC Entry Point)         ║
║                                                                      ║
║  Usage:                                                              ║
║    python main.py               → Waypoint control (default route)   ║
║    python main.py --map         → Map navigator + waypoint control   ║
║    python main.py --lane        → Lane following (default route)     ║
║    python main.py --map --lane  → Map navigator + lane following     ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import sys
import time
import traceback

import carla

from config import WANT_START, WANT_END
from utils.logger import log, sec

from models.connection import connect, load_town10, sync_on, sync_off
from models.vehicle import spawn_tesla, settle_physics, motion_test
from models.route import pick_spawn, snap_end, build_route

from views.map_navigator import MapNavigator
from views.minimap import MiniMap

from controllers.simulation import run, run_lane


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
    nav_waypoints = result.get("waypoints", [])

    log(f"Selected Start: ({start.x:.1f}, {start.y:.1f})")
    log(f"Selected End:   ({end.x:.1f}, {end.y:.1f})")

    # Spawn & vehicle — aynı spawn noktasını kullan
    spawn_tf = pick_spawn(wmap, start, end)
    vehicle = spawn_tesla(world, spawn_tf)
    settle_physics(world, ticks=15)
    time.sleep(0.2)
    motion_test(world, vehicle)

    # Navigator'dan gelen rotayı kullan (haritada görünen rota)
    if len(nav_waypoints) >= 2:
        waypoints = nav_waypoints
        log(f"Using navigator's pre-computed route: {len(waypoints)} waypoints")
    else:
        # Fallback: tekrar hesapla
        end_snapped = snap_end(wmap, end)
        waypoints = build_route(wmap, vehicle.get_location(), end_snapped, world)

    if len(waypoints) < 2:
        print("[ERROR] Could not compute route!")
        vehicle.destroy()
        return

    end_loc = waypoints[-1].transform.location

    # Mini-Map & simulation
    minimap = MiniMap(world)
    
    for _ in range(3):
        world.tick()

    sim_result = run(world, vehicle, waypoints, wmap, end_loc, minimap=minimap)

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


def run_default(client, world, wmap, orig):
    """Test with default (hardcoded) route."""
    vehicle = None

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

        # ── MiniMap ──────────────────────────────────────────────────
        for _ in range(3):
            world.tick()

        # ── Main loop ────────────────────────────────────────────────
        result = run(world, vehicle, waypoints, wmap, end_loc)

        # ── Result ───────────────────────────────────────────────────
        sec("RESULT")
        print(f"  Status : {'SUCCESS ✓' if result['ok'] else 'INCOMPLETE ✗'}")
        print(f"  Time   : {result['t']:.1f}s")
        print(f"  Frames : {result['f']}")

    finally:
        if vehicle:
            try:
                if vehicle.is_alive:
                    vehicle.set_autopilot(False)
                    vehicle.destroy()
                    log("Vehicle destroyed.")
            except Exception:
                pass


# =====================================================================
#  LANE FOLLOWING MODE  (--lane)
# =====================================================================

def run_lane_default(client, world, wmap, orig):
    """Lane following with default (hardcoded) route."""
    vehicle = None

    try:
        # ── Spawn & target ───────────────────────────────────────────
        spawn_tf = pick_spawn(wmap, WANT_START, WANT_END)
        end_loc  = snap_end(wmap, WANT_END)

        # ── Vehicle ──────────────────────────────────────────────────
        vehicle = spawn_tesla(world, spawn_tf)
        settle_physics(world, ticks=15)
        time.sleep(0.2)
        motion_test(world, vehicle)

        # ── Ana döngü (Lane Following) ────────────────────────────────
        # run_lane kendi kamera, detector, controller, dashboard'unu oluşturur
        result = run_lane(world, vehicle, end_loc=end_loc, client=client)

        # ── Sonuç ────────────────────────────────────────────────────
        sec("RESULT")
        print(f"  Status : {'SUCCESS ✓' if result['ok'] else 'INCOMPLETE ✗'}")
        print(f"  Time   : {result['t']:.1f}s")
        print(f"  Frames : {result['f']}")

    finally:
        if vehicle:
            try:
                if vehicle.is_alive:
                    vehicle.set_autopilot(False)
                    vehicle.destroy()
                    log("Vehicle destroyed.")
            except Exception:
                pass


def run_lane_with_map(client, world, wmap, orig):
    """Map navigator + lane following."""
    sec("MAP NAVIGATOR + LANE FOLLOWING MODE")
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

    # Hedef konum
    end_loc = snap_end(wmap, end)

    # Lane following — run_lane kendi bileşenlerini oluşturur
    sim_result = run_lane(world, vehicle, end_loc=end_loc, client=client)

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
    use_map  = "--map" in sys.argv
    use_lane = "--lane" in sys.argv

    # Mode adı belirleme
    if use_lane and use_map:
        mode_name = "MAP + LANE FOLLOWING (OpenCV)"
    elif use_lane:
        mode_name = "LANE FOLLOWING (OpenCV)"
    elif use_map:
        mode_name = "MAP NAVIGATOR (Waypoint PID)"
    else:
        mode_name = "DEFAULT ROUTE (Waypoint PID)"

    print(f"""
╔══════════════════════════════════════════════════════════════════════╗
║  Town10HD Tesla — Autonex Autonomous Driving                        ║
║  Mode: {mode_name:<59s}║
╚══════════════════════════════════════════════════════════════════════╝""")

    try:
        # ── Connection & map ─────────────────────────────────────────
        client = connect()
        world  = load_town10(client)    # Town10HD — modern urban city
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

        # ── Mode routing ─────────────────────────────────────────────
        if use_lane and use_map:
            run_lane_with_map(client, world, wmap, orig)
        elif use_lane:
            run_lane_default(client, world, wmap, orig)
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
