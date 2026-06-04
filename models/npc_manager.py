"""
models/npc_manager.py — NPC Araç Spawn ve Yönetim Sistemi

CARLA Traffic Manager ile NPC araçları spawn eder ve yönetir.
NPC'ler ego araç çevresinde otomatik olarak konumlandırılır.

Kullanım:
    manager = NpcManager(client, world)
    manager.spawn(ego_vehicle, count=8)
    # ... simülasyon döngüsü ...
    manager.destroy_all()
"""

import random
import carla

from config import (
    NPC_COUNT, NPC_SPAWN_RADIUS_M, NPC_MIN_DIST_M,
    NPC_TARGET_SPEED_KMH, NPC_SAFE_DISTANCE_M, NPC_MODELS,
)
from utils.logger import log, sec


class NpcManager:
    """
    NPC araç spawn, Traffic Manager kontrolü ve temizlik yöneticisi.

    Traffic Manager ile NPC'lere:
    - Hedef hız ataması
    - Güvenli takip mesafesi
    - Şerit değiştirme davranışı
    ayarlanır.
    """

    def __init__(self, client, world):
        self._client = client
        self._world = world
        self._actors: list[carla.Actor] = []
        self._traffic_manager = None
        self._tm_port = 8000

        self._init_traffic_manager()

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def spawn(self, ego_vehicle=None, count: int = NPC_COUNT) -> int:
        """
        NPC araçları spawn et.

        Args:
            ego_vehicle: Ego araç — spawn noktaları ego'ya yakın seçilir.
                         None ise harita genelinde rastgele seçilir.
            count: Kaç NPC spawn edilsin.

        Returns:
            Başarıyla spawn edilen NPC sayısı.
        """
        sec("NPC — Spawn")
        blueprint_lib = self._world.get_blueprint_library()
        spawn_points = self._world.get_map().get_spawn_points()

        if not spawn_points:
            log("No spawn points available!", "!")
            return 0

        # Ego araç etrafına yakın spawn noktaları seç
        if ego_vehicle is not None:
            ego_loc = ego_vehicle.get_location()
            spawn_points = sorted(
                spawn_points,
                key=lambda sp: sp.location.distance(ego_loc)
            )
            # Minimum mesafeden uzak, maksimum mesafeden yakın olanları al
            spawn_points = [
                sp for sp in spawn_points
                if NPC_MIN_DIST_M < sp.location.distance(ego_loc) < NPC_SPAWN_RADIUS_M
            ]
            if not spawn_points:
                # Fallback: tüm noktalar
                spawn_points = self._world.get_map().get_spawn_points()
                log("NPC spawn: no points in radius, using all spawn points", "~")

        # Spawn noktalarını karıştır
        random.shuffle(spawn_points)

        spawned = 0
        tried = 0
        max_tries = min(count * 4, len(spawn_points))

        for sp in spawn_points[:max_tries]:
            if spawned >= count:
                break

            # Rastgele blueprint seç
            bp = self._pick_blueprint(blueprint_lib)
            if bp is None:
                tried += 1
                continue

            # Renk ataması
            self._set_random_color(bp)

            actor = self._world.try_spawn_actor(bp, sp)
            if actor is None:
                tried += 1
                continue

            # Traffic Manager ile otonom sürüş
            self._configure_tm(actor)
            self._actors.append(actor)
            spawned += 1

        log(f"NPC spawn tamamlandı: {spawned}/{count} araç (deneme: {tried + spawned})")
        return spawned

    def destroy_all(self):
        """Tüm NPC'leri CARLA'dan kaldır."""
        if not self._actors:
            return

        count = len(self._actors)
        batch = [carla.command.DestroyActor(a.id) for a in self._actors
                 if a.is_alive]
        if batch:
            self._client.apply_batch(batch)

        self._actors.clear()
        log(f"NPC cleanup: {count} araç kaldırıldı.")

    def get_actors(self) -> list:
        """Canlı NPC aktörlerini döndür."""
        return [a for a in self._actors if a.is_alive]

    def get_count(self) -> int:
        """Aktif NPC sayısını döndür."""
        return sum(1 for a in self._actors if a.is_alive)

    def set_speed_for_all(self, speed_kmh: float):
        """Tüm NPC'lerin hedef hızını değiştir."""
        if self._traffic_manager is None:
            return
        for actor in self.get_actors():
            try:
                self._traffic_manager.set_desired_speed(actor, speed_kmh)
            except Exception:
                pass

    def get_nearby_vehicles(self, ego_location, max_dist_m: float = 50.0) -> list:
        """
        Ego araçtan belirli mesafe içindeki NPC'leri döndür.

        Returns:
            List of (actor, distance_m) tuples, sorted by distance.
        """
        result = []
        for actor in self.get_actors():
            try:
                dist = actor.get_location().distance(ego_location)
                if dist <= max_dist_m:
                    result.append((actor, dist))
            except Exception:
                pass
        result.sort(key=lambda x: x[1])
        return result

    # =================================================================
    #  PRIVATE
    # =================================================================

    def _init_traffic_manager(self):
        """Traffic Manager'ı başlat ve senkron moda al."""
        try:
            self._traffic_manager = self._client.get_trafficmanager(self._tm_port)
            self._traffic_manager.set_synchronous_mode(True)
            self._traffic_manager.set_global_distance_to_leading_vehicle(NPC_SAFE_DISTANCE_M)
            log(f"NPC Traffic Manager: port={self._tm_port}, senkron mod aktif")
        except Exception as exc:
            log(f"Traffic Manager başlatılamadı: {exc}", "!")
            self._traffic_manager = None

    def _configure_tm(self, actor: carla.Actor):
        """Tek bir NPC aktörü için Traffic Manager ayarları."""
        if self._traffic_manager is None:
            return
        try:
            actor.set_autopilot(True, self._tm_port)
            self._traffic_manager.set_desired_speed(actor, NPC_TARGET_SPEED_KMH)
            self._traffic_manager.distance_to_leading_vehicle(actor, NPC_SAFE_DISTANCE_M)
            # Şerit değiştirme — gerçekçi ama dikkatli
            self._traffic_manager.auto_lane_change(actor, True)
            self._traffic_manager.random_left_lanechange_percentage(actor, 10)
            self._traffic_manager.random_right_lanechange_percentage(actor, 10)
            # Trafik ışıklarına uy
            self._traffic_manager.ignore_lights_percentage(actor, 0)
            # Hız varyasyonu (± %15 çeşitlilik)
            speed_diff = random.uniform(-15, 15)
            self._traffic_manager.vehicle_percentage_speed_difference(actor, speed_diff)
        except Exception as exc:
            log(f"NPC TM config hatası (ID={actor.id}): {exc}", "!")

    def _pick_blueprint(self, blueprint_lib) -> carla.ActorBlueprint | None:
        """Yapılandırılmış modeller listesinden rastgele blueprint seç."""
        random.shuffle(NPC_MODELS)
        for model in NPC_MODELS:
            try:
                bp = blueprint_lib.find(model)
                if bp is not None:
                    return bp
            except Exception:
                continue

        # Fallback: herhangi bir araç
        vehicles = blueprint_lib.filter("vehicle.*")
        return random.choice(vehicles) if vehicles else None

    @staticmethod
    def _set_random_color(bp: carla.ActorBlueprint):
        """Araç rengi rastgele ata (ego kırmızı'dan ayırt etmek için)."""
        if not bp.has_attribute("color"):
            return
        colors = [
            "0,128,255",    # mavi
            "255,200,0",    # sarı
            "0,200,80",     # yeşil
            "200,200,200",  # gümüş
            "80,40,0",      # kahve
            "180,100,20",   # turuncu
            "200,0,200",    # mor
        ]
        bp.set_attribute("color", random.choice(colors))
