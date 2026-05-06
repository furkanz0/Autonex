"""
models/connection.py — CARLA connection, map loading and sync mode management
"""

import time
import carla

from config import HOST, PORT, TIMEOUT_S, FIXED_DELTA
from utils.logger import log, sec


def connect():
    """Connect to the CARLA server and return client."""
    sec("1 – CARLA Connection")
    c = carla.Client(HOST, PORT)
    c.set_timeout(TIMEOUT_S)
    log(f"Connected {HOST}:{PORT}  server={c.get_server_version()}")
    return c


def load_town04(client):
    """Load Town04 map (skip if already loaded)."""
    sec("2 – Town04")
    world = client.get_world()
    if "Town04" not in world.get_map().name:
        print("  [~] Loading Town04...")
        world = client.load_world("Town04")
        time.sleep(3)
    log(f"Map: {world.get_map().name}")
    return world


def sync_on(world):
    """Enable synchronous mode, return original settings."""
    orig = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode    = True
    s.fixed_delta_seconds = FIXED_DELTA
    s.no_rendering_mode   = False
    world.apply_settings(s)
    log(f"Sync mode ON  Δt={FIXED_DELTA}s")
    return orig


def sync_off(world, original_settings):
    """Restore original (async) settings."""
    world.apply_settings(original_settings)
    log("Switched back to async mode.")
