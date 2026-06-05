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
    NPC_WALKERS_ENABLED,
    NPC_WALKER_COUNT,
    NPC_WALKER_MIN_EGO_SPAWN_DISTANCE_M,
    NPC_WALKER_SPAWN_RADIUS_M,
    NPC_WALKER_AHEAD_DISTANCE_M,
    NPC_WALKER_SIDE_DISTANCE_M,
    NPC_WALKER_CROSSWALK_RATIO,
    NPC_WALKER_RUNNING_PERCENT,
    NPC_WALKER_CROSSING_FACTOR,
)
from utils.logger import log, sec


def spawn_light_traffic(client, world, ego_vehicle, count=NPC_TRAFFIC_COUNT):
    """Spawn Traffic Manager controlled NPC vehicles and AI pedestrians."""
    if (
        (not NPC_TRAFFIC_ENABLED or count <= 0)
        and (not NPC_WALKERS_ENABLED or NPC_WALKER_COUNT <= 0)
    ):
        return []

    sec("NPC Traffic")
    rng = random.Random(NPC_TRAFFIC_SEED)
    ego_loc = ego_vehicle.get_location() if ego_vehicle is not None else None
    ego_tf = ego_vehicle.get_transform() if ego_vehicle is not None else None
    actors = []

    if NPC_TRAFFIC_ENABLED and count > 0:
        actors.extend(_spawn_npc_vehicles(client, world, ego_loc, count, rng))

    if NPC_WALKERS_ENABLED and NPC_WALKER_COUNT > 0:
        actors.extend(_spawn_npc_walkers(
            client, world, ego_loc, ego_tf, NPC_WALKER_COUNT, rng))

    for _ in range(NPC_SPAWN_SETTLE_TICKS):
        world.tick()

    vehicle_count = sum(1 for actor in actors if _actor_type(actor).startswith("vehicle."))
    walker_count = sum(1 for actor in actors if _actor_type(actor).startswith("walker.pedestrian."))
    log(f"Spawned {vehicle_count} NPC vehicles and {walker_count} NPC pedestrians")
    return actors


def _spawn_npc_vehicles(client, world, ego_loc, count, rng):
    """Spawn a small, Traffic Manager controlled NPC vehicle fleet."""
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
        _safe_tm_call(traffic_manager, "ignore_walkers_percentage", actor, 0)
        _safe_tm_call(
            traffic_manager, "vehicle_percentage_speed_difference",
            actor, NPC_SPEED_DIFF_PERCENT + rng.uniform(-5.0, 5.0))
        actors.append(actor)

    log(f"Spawned {len(actors)} NPC vehicles (light traffic)")
    return actors


def _spawn_npc_walkers(client, world, ego_loc, ego_tf, count, rng):
    """Spawn visible pedestrians on sidewalks and crosswalks."""
    blueprint_lib = world.get_blueprint_library()
    walker_blueprints = list(blueprint_lib.filter("walker.pedestrian.*"))
    try:
        controller_bp = blueprint_lib.find("controller.ai.walker")
    except Exception:
        controller_bp = None

    if not walker_blueprints:
        log("No pedestrian walker blueprints found", "!")
        return []

    _safe_world_call(world, "set_pedestrians_cross_factor", NPC_WALKER_CROSSING_FACTOR)
    walkers = []
    controllers = []
    plans = _walker_spawn_plans(world, ego_loc, ego_tf, count, rng)
    log(
        f"Walker spawn plans: {len(plans)} "
        f"(blueprints={len(walker_blueprints)}, controller={'yes' if controller_bp else 'no'})")
    crosswalk_spawned = 0
    sidewalk_spawned = 0

    spawned_specs = _spawn_walkers_batch(client, world, walker_blueprints, plans, rng, count)
    for walker, loc, destination, source, speed in spawned_specs:
        controller = None
        if controller_bp is not None:
            controller = world.try_spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
            if controller is not None:
                try:
                    controller.start()
                    controller.go_to_location(destination)
                    controller.set_max_speed(speed)
                except Exception:
                    try:
                        controller.destroy()
                    except Exception:
                        pass
                    controller = None

        walkers.append(walker)
        if controller and controller.is_alive:
            controllers.append(controller)
        if source == "crosswalk":
            crosswalk_spawned += 1
        else:
            sidewalk_spawned += 1

    log(
        f"Spawned {len(walkers)} NPC pedestrians "
        f"({crosswalk_spawned} crosswalk, {sidewalk_spawned} sidewalk)")
    return controllers + walkers


def _spawn_walkers_batch(client, world, walker_blueprints, plans, rng, count):
    commands = []
    specs = []
    for loc, destination, source in plans:
        if len(commands) >= count:
            break
        walker_bp = rng.choice(walker_blueprints)
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "false")
        speed = _walker_speed(walker_bp, rng)

        for transform in _walker_spawn_transforms(loc, destination, source):
            commands.append(carla.command.SpawnActor(walker_bp, transform))
            specs.append((loc, destination, source, speed))
            break

    if not commands:
        return []

    spawned = []
    try:
        responses = client.apply_batch_sync(commands, True)
    except Exception as exc:
        log(f"Walker batch spawn failed: {exc}", "!")
        responses = []

    for response, spec in zip(responses, specs):
        if getattr(response, "error", None):
            continue
        try:
            actor = world.get_actor(response.actor_id)
        except Exception:
            actor = None
        if actor is not None:
            spawned.append((actor, spec[0], spec[1], spec[2], spec[3]))

    if len(spawned) < min(count, len(plans)):
        # Last-resort direct spawn, kept for older CARLA builds where batch walker
        # spawning is flaky or only partially succeeds.
        for loc, destination, source in plans:
            if len(spawned) >= count:
                break
            walker_bp = rng.choice(walker_blueprints)
            if walker_bp.has_attribute("is_invincible"):
                walker_bp.set_attribute("is_invincible", "false")
            walker = _try_spawn_walker(world, walker_bp, loc, destination, source)
            if walker is None:
                continue
            spawned.append((walker, loc, destination, source, _walker_speed(walker_bp, rng)))

    log(f"Walker actors actually spawned: {len(spawned)}/{min(count, len(plans))}")
    return spawned


def _walker_spawn_plans(world, ego_loc, ego_tf, count, rng):
    plans = []
    crosswalk_target = int(count * NPC_WALKER_CROSSWALK_RATIO)
    plans.extend(_crosswalk_spawn_plans(world, ego_loc, ego_tf, crosswalk_target, rng))
    nearby_target = min(12, max(0, count - len(plans)))
    plans.extend(_nearby_navmesh_sidewalk_spawn_plans(world, ego_loc, nearby_target, rng))
    nearby_strict_target = min(8, max(0, count - len(plans)))
    plans.extend(_nearby_sidewalk_spawn_plans(world, ego_loc, nearby_strict_target, rng))
    plans.extend(_global_sidewalk_spawn_plans(world, count - len(plans), rng))

    if len(plans) < count:
        plans.extend(_nav_sidewalk_spawn_plans(world, ego_loc, ego_tf, count - len(plans), rng))

    rng.shuffle(plans)
    return plans[:count * 2]


def _crosswalk_spawn_plans(world, ego_loc, ego_tf, count, rng):
    if count <= 0:
        return []

    try:
        crosswalk_points = list(world.get_map().get_crosswalks())
    except Exception:
        crosswalk_points = []

    if not crosswalk_points:
        return []

    points = [
        p for p in crosswalk_points
        if _crosswalk_allows_pedestrians_now(world, p)
    ]
    rng.shuffle(points)
    points = points[:max(count * 10, 40)]

    plans = []
    used = []
    for point in points:
        if len(plans) >= count:
            break
        if any(point.distance(prev) < 2.0 for prev in used):
            continue

        crossing = _crosswalk_crossing_pair(crosswalk_points, point, rng)
        if crossing is None:
            continue
        else:
            spawn_loc, destination = crossing

        plans.append((spawn_loc, destination, "crosswalk"))
        used.append(spawn_loc)

    return plans


def _crosswalk_crossing_pair(crosswalk_points, point, rng):
    """Return start/end points that both lie on CARLA crosswalk geometry."""
    candidates = [
        p for p in crosswalk_points
        if 2.5 <= p.distance(point) <= 9.0
        and _crosswalk_segment_stays_on_geometry(crosswalk_points, point, p)
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda p: p.distance(point), reverse=True)
    destination = candidates[min(len(candidates) - 1, rng.randrange(min(4, len(candidates))))] 
    return point, destination


def _crosswalk_segment_stays_on_geometry(crosswalk_points, start, end):
    mid = carla.Location(
        x=(start.x + end.x) * 0.5,
        y=(start.y + end.y) * 0.5,
        z=(start.z + end.z) * 0.5)
    return any(p.distance(mid) <= 2.2 for p in crosswalk_points)


def _crosswalk_allows_pedestrians_now(world, point):
    """Allow crossing at uncontrolled crosswalks or while nearby vehicle lights are red."""
    try:
        traffic_lights = list(world.get_actors().filter("traffic.traffic_light"))
    except Exception:
        return True

    nearby_lights = []
    for light in traffic_lights:
        try:
            if light.get_location().distance(point) <= 28.0:
                nearby_lights.append(light)
        except Exception:
            continue

    if not nearby_lights:
        return True

    for light in nearby_lights:
        try:
            state = light.get_state()
        except Exception:
            continue
        if state in (carla.TrafficLightState.Green, carla.TrafficLightState.Yellow):
            return False

    return any(
        _safe_traffic_light_state(light) == carla.TrafficLightState.Red
        for light in nearby_lights)


def _safe_traffic_light_state(light):
    try:
        return light.get_state()
    except Exception:
        return None


def _nearby_navmesh_sidewalk_spawn_plans(world, ego_loc, count, rng):
    """Use CARLA pedestrian navmesh near ego so walkers actually spawn visibly."""
    if count <= 0 or ego_loc is None:
        return []

    plans = []
    tries = max(count * 30, 180)
    for _ in range(tries):
        if len(plans) >= count:
            break

        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        dist = loc.distance(ego_loc)
        if dist < 8.0 or dist > 80.0:
            continue

        destination = _nearby_navmesh_destination(world, loc, rng)
        if destination is None:
            continue
        plans.append((loc, destination, "nav_sidewalk"))

    return plans


def _nearby_sidewalk_spawn_plans(world, ego_loc, count, rng):
    """Keep a few legal sidewalk pedestrians visible near the ego route."""
    if count <= 0 or ego_loc is None:
        return []

    try:
        wmap = world.get_map()
        base_wp = wmap.get_waypoint(
            ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    except Exception:
        base_wp = None
    if base_wp is None:
        return []

    plans = []
    distances = [8.0, 14.0, 22.0, 32.0, 46.0, 62.0, 82.0]
    for distance in distances:
        if len(plans) >= count:
            break
        try:
            candidates = base_wp.next(distance)
            wp = candidates[0] if candidates else None
        except Exception:
            wp = None
        if wp is None:
            continue

        sides = ["left", "right"]
        rng.shuffle(sides)
        for side in sides:
            if len(plans) >= count:
                break
            sidewalk_wp = _sidewalk_lane_from(wp, side)
            pair = _sidewalk_walk_pair(sidewalk_wp, rng)
            if pair is None:
                continue
            plans.append((pair[0], pair[1], "sidewalk"))

    return plans


def _global_sidewalk_spawn_plans(world, count, rng):
    """Distribute walkers across the map using only CARLA Sidewalk lanes."""
    if count <= 0:
        return []

    try:
        wmap = world.get_map()
        spawn_points = list(wmap.get_spawn_points())
    except Exception:
        spawn_points = []
    if not spawn_points:
        return []

    rng.shuffle(spawn_points)
    plans = []
    for spawn_tf in spawn_points:
        if len(plans) >= count:
            break
        try:
            wp = wmap.get_waypoint(
                spawn_tf.location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving)
        except Exception:
            wp = None
        if wp is None:
            continue

        for side in ("left", "right"):
            if len(plans) >= count:
                break
            sidewalk_wp = _sidewalk_lane_from(wp, side)
            if sidewalk_wp is None:
                continue

            pair = _sidewalk_walk_pair(sidewalk_wp, rng)
            if pair is not None:
                plans.append((pair[0], pair[1], "sidewalk"))

    return plans


def _nav_sidewalk_spawn_plans(world, ego_loc, ego_tf, count, rng):
    plans = []
    try:
        wmap = world.get_map()
    except Exception:
        return plans

    tries = max(count * 40, 160)
    for _ in range(tries):
        if len(plans) >= count:
            break

        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue

        destination = _nearby_navmesh_destination(world, loc, rng)
        if destination is not None:
            plans.append((loc, destination, "nav_sidewalk"))

    return plans


def _nearby_navmesh_destination(world, origin, rng):
    best = None
    for _ in range(24):
        dest = world.get_random_location_from_navigation()
        if dest is None:
            continue
        dist = origin.distance(dest)
        if 5.0 <= dist <= 18.0:
            return dest
        if best is None or abs(dist - 10.0) < best[0]:
            best = (abs(dist - 10.0), dest)
    return best[1] if best else None


def _sidewalk_lane_from(wp, side):
    getter = "get_left_lane" if side == "left" else "get_right_lane"
    current = wp
    for _ in range(5):
        try:
            current = getattr(current, getter)()
        except Exception:
            return None
        if current is None:
            return None
        try:
            if current.lane_type == carla.LaneType.Sidewalk:
                return current
        except Exception:
            return None
    return None


def _sidewalk_walk_pair(sidewalk_wp, rng):
    if sidewalk_wp is None:
        return None

    try:
        distance = rng.uniform(6.0, 14.0)
        candidates = sidewalk_wp.next(distance)
    except Exception:
        candidates = []
    if not candidates:
        return None

    rng.shuffle(candidates)
    for dest_wp in candidates:
        try:
            if dest_wp.lane_type != carla.LaneType.Sidewalk:
                continue
            start = carla.Location(
                sidewalk_wp.transform.location.x,
                sidewalk_wp.transform.location.y,
                sidewalk_wp.transform.location.z + 0.35)
            destination = carla.Location(
                dest_wp.transform.location.x,
                dest_wp.transform.location.y,
                dest_wp.transform.location.z + 0.35)
            if start.distance(destination) >= 4.0:
                return start, destination
        except Exception:
            continue
    return None


def _try_spawn_walker(world, walker_bp, loc, destination, source):
    candidates = [loc]
    if source == "crosswalk":
        try:
            candidates.extend([
                loc + (destination - loc) * 0.10,
                loc + (destination - loc) * 0.20,
            ])
        except Exception:
            pass

    for candidate in candidates:
        for z_offset in (0.0, 0.25, 0.55, 1.0):
            try:
                spawn_loc = carla.Location(
                    candidate.x,
                    candidate.y,
                    candidate.z + z_offset)
                walker = world.try_spawn_actor(walker_bp, carla.Transform(spawn_loc))
                if walker is not None:
                    return walker
            except Exception:
                continue
    return None


def _walker_spawn_transforms(loc, destination, source):
    candidates = [loc]
    if source == "crosswalk":
        try:
            candidates.extend([
                loc + (destination - loc) * 0.10,
                loc + (destination - loc) * 0.20,
            ])
        except Exception:
            pass

    transforms = []
    for candidate in candidates:
        for z_offset in (0.0, 0.25, 0.55, 1.0):
            spawn_loc = carla.Location(
                candidate.x,
                candidate.y,
                candidate.z + z_offset)
            transforms.append(carla.Transform(spawn_loc))
    return transforms


def _is_useful_walker_location(loc, ego_loc, ego_tf):
    if ego_loc is None:
        return True

    dist = loc.distance(ego_loc)
    if dist < NPC_WALKER_MIN_EGO_SPAWN_DISTANCE_M or dist > NPC_WALKER_SPAWN_RADIUS_M:
        return False
    if ego_tf is None:
        return True

    forward = ego_tf.get_forward_vector()
    right = ego_tf.get_right_vector()
    delta = loc - ego_loc
    ahead = delta.x * forward.x + delta.y * forward.y + delta.z * forward.z
    side = abs(delta.x * right.x + delta.y * right.y + delta.z * right.z)

    return -12.0 <= ahead <= NPC_WALKER_AHEAD_DISTANCE_M and side <= NPC_WALKER_SIDE_DISTANCE_M


def destroy_traffic(actors):
    """Disable NPC behavior and destroy spawned vehicles, walkers and controllers."""
    if not actors:
        return

    controllers = []
    others = []
    for actor in actors:
        if _actor_type(actor).startswith("controller.ai.walker"):
            controllers.append(actor)
        else:
            others.append(actor)

    destroyed = 0
    for controller in controllers:
        try:
            if controller and controller.is_alive:
                controller.stop()
        except Exception:
            pass

    for actor in controllers + others:
        try:
            if actor and actor.is_alive:
                if _actor_type(actor).startswith("vehicle."):
                    actor.set_autopilot(False)
                actor.destroy()
                destroyed += 1
        except Exception:
            pass

    log(f"Destroyed {destroyed} NPC actors.")


def _walker_speed(walker_bp, rng):
    if not walker_bp.has_attribute("speed"):
        return 1.4

    speeds = walker_bp.get_attribute("speed").recommended_values
    if not speeds:
        return 1.4

    speed_idx = 2 if len(speeds) > 2 and rng.random() < NPC_WALKER_RUNNING_PERCENT else 1
    speed_idx = min(speed_idx, len(speeds) - 1)
    try:
        return float(speeds[speed_idx])
    except (TypeError, ValueError):
        return 1.4


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


def _safe_world_call(world, method_name, *args):
    try:
        method = getattr(world, method_name)
        method(*args)
    except Exception:
        pass


def _actor_type(actor):
    try:
        return actor.type_id
    except Exception:
        return ""
