"""
models/traffic.py - Light NPC traffic helpers.
"""

import random

import carla

from config import (
    NPC_TRAFFIC_ENABLED,
    NPC_TRAFFIC_COUNT,
    NPC_TRAFFIC_SEED,
    NPC_MIN_EGO_SPAWN_DISTANCE_M,
    NPC_SPEED_DIFF_PERCENT,
    NPC_FOLLOW_DISTANCE_M,
    NPC_SPAWN_SETTLE_TICKS,
)
from utils.logger import log, sec


def spawn_light_traffic(client, world, ego_vehicle, count=NPC_TRAFFIC_COUNT):
    """Spawn a small, Traffic Manager controlled NPC fleet."""
    if not NPC_TRAFFIC_ENABLED or count <= 0:
        return []

    sec("NPC Traffic")
    rng = random.Random(NPC_TRAFFIC_SEED)
    traffic_manager = client.get_trafficmanager()
    _safe_tm_call(traffic_manager, "set_synchronous_mode", True)
    _safe_tm_call(
        traffic_manager, "set_global_distance_to_leading_vehicle",
        NPC_FOLLOW_DISTANCE_M)
    _safe_tm_call(
        traffic_manager, "global_percentage_speed_difference",
        NPC_SPEED_DIFF_PERCENT)

    blueprints = _vehicle_blueprints(world)
    spawn_points = list(world.get_map().get_spawn_points())
    rng.shuffle(spawn_points)

    ego_loc = ego_vehicle.get_location() if ego_vehicle is not None else None
    actors = []

    for spawn_tf in spawn_points:
        if len(actors) >= count:
            break
        if ego_loc and spawn_tf.location.distance(ego_loc) < NPC_MIN_EGO_SPAWN_DISTANCE_M:
            continue

        bp = rng.choice(blueprints)
        _set_random_color(bp, rng)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "npc")

        actor = world.try_spawn_actor(bp, spawn_tf)
        if actor is None:
            continue

        actor.set_autopilot(True, traffic_manager.get_port())
        _safe_tm_call(traffic_manager, "auto_lane_change", actor, False)
        _safe_tm_call(traffic_manager, "random_left_lanechange_percentage", actor, 0)
        _safe_tm_call(traffic_manager, "random_right_lanechange_percentage", actor, 0)
        _safe_tm_call(
            traffic_manager, "distance_to_leading_vehicle",
            actor, NPC_FOLLOW_DISTANCE_M)
        _safe_tm_call(traffic_manager, "ignore_lights_percentage", actor, 0)
        _safe_tm_call(traffic_manager, "ignore_signs_percentage", actor, 0)
        _safe_tm_call(
            traffic_manager, "vehicle_percentage_speed_difference",
            actor, NPC_SPEED_DIFF_PERCENT + rng.uniform(-5.0, 5.0))
        actors.append(actor)

    for _ in range(NPC_SPAWN_SETTLE_TICKS):
        world.tick()

    log(f"Spawned {len(actors)} NPC vehicles (light traffic)")
    return actors


def destroy_traffic(actors):
    """Disable autopilot and destroy spawned NPC vehicles."""
    if not actors:
        return

    destroyed = 0
    for actor in actors:
        try:
            if actor and actor.is_alive:
                actor.set_autopilot(False)
                actor.destroy()
                destroyed += 1
        except Exception:
            pass

    log(f"Destroyed {destroyed} NPC vehicles.")


def _vehicle_blueprints(world):
    blueprints = []
    for bp in world.get_blueprint_library().filter("vehicle.*"):
        if not bp.has_attribute("number_of_wheels"):
            continue
        if bp.get_attribute("number_of_wheels").as_int() != 4:
            continue
        bp_id = bp.id.lower()
        if "carlamotors" in bp_id or "firetruck" in bp_id or "ambulance" in bp_id:
            continue
        blueprints.append(bp)

    if not blueprints:
        raise RuntimeError("No suitable 4-wheel vehicle blueprints found")
    return blueprints


def _set_random_color(bp, rng):
    if not bp.has_attribute("color"):
        return
    colors = bp.get_attribute("color").recommended_values
    if colors:
        bp.set_attribute("color", rng.choice(colors))


def _safe_tm_call(traffic_manager, method_name, *args):
    try:
        method = getattr(traffic_manager, method_name)
        method(*args)
    except Exception:
        pass
