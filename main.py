"""
main.py - CARLA Town10HD Tesla Simulation entry point.

Usage:
    python main.py
    python main.py --map
    python main.py --lane
    python main.py --map --lane
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
from models.traffic import spawn_light_traffic, destroy_traffic

from views.map_navigator import MapNavigator
from views.minimap import MiniMap

from controllers.simulation import run, run_lane


def run_with_map(client, world, wmap, orig):
    """Interactive route selection via map navigator and waypoint control."""
    sec("MAP NAVIGATOR MODE")
    vehicle = None
    traffic_actors = []
    nav = MapNavigator(world)
    result = nav.run()

    if result is None:
        print("  [!] Exited map navigator.")
        return

    start = result["start"]
    end = result["end"]
    route_end = result.get("route_end", end)
    nav_waypoints = result.get("waypoints", [])

    log(f"Selected Start: ({start.x:.1f}, {start.y:.1f})")
    log(f"Selected End:   ({end.x:.1f}, {end.y:.1f})")

    try:
        spawn_tf = pick_spawn(wmap, start, end)
        vehicle = spawn_tesla(world, spawn_tf)
        settle_physics(world, ticks=15)
        time.sleep(0.2)
        motion_test(world, vehicle)
        traffic_actors = spawn_light_traffic(client, world, vehicle)

        if len(nav_waypoints) >= 2:
            waypoints = nav_waypoints
            log(f"Using navigator's pre-computed route: {len(waypoints)} waypoints")
        else:
            end_snapped = snap_end(wmap, route_end)
            waypoints = build_route(wmap, vehicle.get_location(), end_snapped, world)

        if len(waypoints) < 2:
            print("[ERROR] Could not compute route!")
            return

        end_loc = snap_end(wmap, route_end)
        minimap = MiniMap(world)

        for _ in range(3):
            world.tick()

        sim_result = run(world, vehicle, waypoints, wmap, end_loc, minimap=minimap)
        _print_result(sim_result)

    finally:
        destroy_traffic(traffic_actors)
        _destroy_vehicle(vehicle)


def run_default(client, world, wmap, orig):
    """Run the default hardcoded route with waypoint control."""
    vehicle = None
    traffic_actors = []

    try:
        spawn_tf = pick_spawn(wmap, WANT_START, WANT_END)
        end_loc = snap_end(wmap, WANT_END)
        vehicle = spawn_tesla(world, spawn_tf)
        settle_physics(world, ticks=15)
        time.sleep(0.2)
        motion_test(world, vehicle)
        traffic_actors = spawn_light_traffic(client, world, vehicle)

        waypoints = build_route(wmap, vehicle.get_location(), end_loc, world)
        if len(waypoints) < 2:
            print("[ERROR] Could not compute route!")
            sys.exit(1)

        for _ in range(3):
            world.tick()

        result = run(world, vehicle, waypoints, wmap, end_loc)
        _print_result(result)

    finally:
        destroy_traffic(traffic_actors)
        _destroy_vehicle(vehicle)


def run_lane_default(client, world, wmap, orig):
    """Run the default hardcoded route with lane following."""
    vehicle = None
    traffic_actors = []

    try:
        spawn_tf = pick_spawn(wmap, WANT_START, WANT_END)
        end_loc = snap_end(wmap, WANT_END)
        vehicle = spawn_tesla(world, spawn_tf)
        settle_physics(world, ticks=15)
        time.sleep(0.2)
        motion_test(world, vehicle)
        traffic_actors = spawn_light_traffic(client, world, vehicle)

        waypoints = build_route(wmap, vehicle.get_location(), end_loc, world)
        if len(waypoints) < 2:
            waypoints = None
            log("Lane mode route lock disabled: route could not be computed", "!")
        else:
            log(f"Lane mode route lock enabled: {len(waypoints)} waypoints")

        result = run_lane(
            world,
            vehicle,
            end_loc=end_loc,
            client=client,
            waypoints=waypoints,
        )
        _print_result(result)

    finally:
        destroy_traffic(traffic_actors)
        _destroy_vehicle(vehicle)


def run_lane_with_map(client, world, wmap, orig):
    """Interactive route selection via map navigator and lane following."""
    sec("MAP NAVIGATOR + LANE FOLLOWING MODE")
    vehicle = None
    traffic_actors = []
    nav = MapNavigator(world)
    result = nav.run()

    if result is None:
        print("  [!] Exited map navigator.")
        return

    start = result["start"]
    end = result["end"]
    route_end = result.get("route_end", end)
    nav_waypoints = result.get("waypoints", [])

    log(f"Selected Start: ({start.x:.1f}, {start.y:.1f})")
    log(f"Selected End:   ({end.x:.1f}, {end.y:.1f})")

    try:
        spawn_tf = pick_spawn(wmap, start, end)
        vehicle = spawn_tesla(world, spawn_tf)
        settle_physics(world, ticks=15)
        time.sleep(0.2)
        motion_test(world, vehicle)
        traffic_actors = spawn_light_traffic(client, world, vehicle)

        end_loc = snap_end(wmap, route_end)
        if len(nav_waypoints) >= 2:
            waypoints = nav_waypoints
            log(f"Using navigator route for lane lock: {len(waypoints)} waypoints")
        else:
            waypoints = build_route(wmap, vehicle.get_location(), end_loc, world)
            log(f"Computed fallback lane lock route: {len(waypoints)} waypoints")

        if len(waypoints) < 2:
            print("[ERROR] Could not compute lane lock route!")
            return

        minimap = MiniMap(world)
        for _ in range(3):
            world.tick()

        sim_result = run_lane(
            world,
            vehicle,
            end_loc=end_loc,
            client=client,
            waypoints=waypoints,
            minimap=minimap,
        )
        _print_result(sim_result)

    finally:
        destroy_traffic(traffic_actors)
        _destroy_vehicle(vehicle)


def _print_result(result):
    sec("RESULT")
    print(f"  Status : {'SUCCESS' if result['ok'] else 'INCOMPLETE'}")
    print(f"  Time   : {result['t']:.1f}s")
    print(f"  Frames : {result['f']}")


def _destroy_vehicle(vehicle):
    try:
        if vehicle and vehicle.is_alive:
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

    if use_lane and use_map:
        mode_name = "MAP + LANE FOLLOWING"
    elif use_lane:
        mode_name = "LANE FOLLOWING"
    elif use_map:
        mode_name = "MAP NAVIGATOR"
    else:
        mode_name = "DEFAULT ROUTE"

    print(f"\nAutonex Autonomous Driving - Mode: {mode_name}\n")

    try:
        client = connect()
        world = load_town10(client)
        wmap = world.get_map()
        orig = sync_on(world)

        world.set_weather(carla.WeatherParameters.ClearNoon)
        log("Weather: ClearNoon")

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
