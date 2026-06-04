"""
controllers/acc_controller.py — Adaptive Cruise Control (ACC)

Araç tespiti sonuçlarını alır, throttle/brake değerleri üretir.

Durum Makinesi:
    FREE_DRIVE  → Önde araç yok — hedef hızda seyret
    FOLLOWING   → Güvenli mesafede araç var — hızını eşitle
    BRAKING     → Kritik mesafede araç — fren artır
    EMERGENCY   → Çok yakın — tam fren

Kullanım:
    acc = AccController(target_speed_kmh=30.0)
    decision = acc.compute(vehicle_result, current_speed_kmh, last_steer)
    vehicle.apply_control(carla.VehicleControl(
        throttle=decision.throttle,
        brake=decision.brake,
        steer=decision.steer
    ))
"""

from dataclasses import dataclass
from enum import Enum, auto

from config import (
    VD_SAFE_DIST_M, VD_CRITICAL_DIST_M, VD_EMERGENCY_DIST_M,
    ACC_FOLLOW_KP, ACC_MAX_THROTTLE, ACC_BRAKE_KP, ACC_MAX_BRAKE,
    ACC_CREEP_THROTTLE, TARGET_SPEED_KMH,
)


# =====================================================================
#  ENUM: ACC DURUM MAKİNESİ
# =====================================================================

class ACCState(Enum):
    FREE_DRIVE = auto()   # Serbest seyir — hedef hızda git
    FOLLOWING  = auto()   # Takip modu — hızı eşitle
    BRAKING    = auto()   # Fren modu — yavaşla
    EMERGENCY  = auto()   # Acil fren — tam dur


# =====================================================================
#  ÇIKTI VERİ YAPISI
# =====================================================================

@dataclass
class ACCDecision:
    """ACC'nin ürettiği kontrol kararı."""
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    state: ACCState = ACCState.FREE_DRIVE
    active: bool = False         # ACC devrede mi?
    closest_dist_m: float = float("inf")
    reason: str = ""             # Loglama / debug için

    @property
    def state_name(self) -> str:
        return self.state.name


# =====================================================================
#  ANA SINIF
# =====================================================================

class AccController:
    """
    Adaptive Cruise Control — araç tespiti + PID tabanlı hız kontrolü.

    Mevcut traffic light + lane kontrolüne paralel çalışır.
    Sadece araç tespit edildiğinde devreye girer.
    """

    def __init__(self, target_speed_kmh: float = TARGET_SPEED_KMH):
        self._target_kmh = target_speed_kmh
        self._state = ACCState.FREE_DRIVE
        self._smooth_throttle = 0.0   # yumuşatılmış gaz
        self._smooth_brake = 0.0      # yumuşatılmış fren
        self._alpha = 0.25            # EMA yumuşatma faktörü

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def compute(self, detection_result, current_speed_kmh: float,
                last_steer: float = 0.0) -> ACCDecision:
        """
        ACC kararını hesapla.

        Args:
            detection_result: VehicleDetectionResult (models/vehicle_detector.py)
            current_speed_kmh: Aracın anlık hızı (km/h)
            last_steer: Son direksiyon değeri (koruma amaçlı)

        Returns:
            ACCDecision
        """
        # Araç tespit edilmedi → serbest seyir
        if not detection_result.detected:
            return self._free_drive(current_speed_kmh, last_steer)

        closest_dist = detection_result.closest_distance_m

        # ── Durum geçiş makinesi ─────────────────────────────────────
        if closest_dist < VD_EMERGENCY_DIST_M:
            return self._emergency(closest_dist, last_steer)

        if closest_dist < VD_CRITICAL_DIST_M:
            return self._braking(closest_dist, current_speed_kmh, last_steer)

        if closest_dist < VD_SAFE_DIST_M:
            return self._following(closest_dist, current_speed_kmh, last_steer)

        # Güvenli mesafenin dışında — serbest seyir
        return self._free_drive(current_speed_kmh, last_steer)

    @property
    def state(self) -> ACCState:
        return self._state

    def set_target_speed(self, speed_kmh: float):
        self._target_kmh = speed_kmh

    # =================================================================
    #  DURUM HESAPLAMALARI
    # =================================================================

    def _free_drive(self, speed: float, steer: float) -> ACCDecision:
        """Önde araç yok — hedef hızda seyret."""
        self._state = ACCState.FREE_DRIVE
        return ACCDecision(
            throttle=0.0,  # Normal kontrolcüye bırak
            brake=0.0,
            steer=steer,
            state=ACCState.FREE_DRIVE,
            active=False,
            reason="FREE: araç tespit yok",
        )

    def _following(self, dist: float, speed: float, steer: float) -> ACCDecision:
        """
        Güvenli mesafede takip — hız orantılı azaltma.

        Mesafe azaldıkça gaz azalır, tam güvenli mesafede hedef hız korunur.
        """
        self._state = ACCState.FOLLOWING

        # Güvenlik oranı: 0 (çok yakın) → 1 (tam güvenli)
        safety_ratio = (dist - VD_CRITICAL_DIST_M) / (VD_SAFE_DIST_M - VD_CRITICAL_DIST_M)
        safety_ratio = max(0.0, min(1.0, safety_ratio))

        # Hedef throttle: güvenlik oranına göre ölçekle
        target_throttle = ACC_MAX_THROTTLE * safety_ratio

        # Hız çok yüksekse hafif fren
        target_brake = 0.0
        if speed > self._target_kmh * safety_ratio + 5:
            target_brake = 0.15

        throttle = self._smooth(self._smooth_throttle, target_throttle)
        brake = self._smooth(self._smooth_brake, target_brake)
        self._smooth_throttle = throttle
        self._smooth_brake = brake

        return ACCDecision(
            throttle=throttle,
            brake=brake,
            steer=steer,
            state=ACCState.FOLLOWING,
            active=True,
            closest_dist_m=dist,
            reason=f"FOLLOW: {dist:.1f}m (safety={safety_ratio:.2f})",
        )

    def _braking(self, dist: float, speed: float, steer: float) -> ACCDecision:
        """
        Kritik mesafede fren — orantılı frenleme.

        Mesafe azaldıkça fren artar.
        """
        self._state = ACCState.BRAKING

        # Fren oranı: 0 (kritik sınırda) → 1 (acil sınırda)
        brake_ratio = 1.0 - (dist - VD_EMERGENCY_DIST_M) / (VD_CRITICAL_DIST_M - VD_EMERGENCY_DIST_M)
        brake_ratio = max(0.0, min(1.0, brake_ratio))

        target_brake = ACC_BRAKE_KP * 5 + ACC_MAX_BRAKE * brake_ratio * 0.7

        throttle = self._smooth(self._smooth_throttle, 0.0)
        brake = self._smooth(self._smooth_brake, min(target_brake, ACC_MAX_BRAKE))
        self._smooth_throttle = throttle
        self._smooth_brake = brake

        return ACCDecision(
            throttle=throttle,
            brake=brake,
            steer=steer,
            state=ACCState.BRAKING,
            active=True,
            closest_dist_m=dist,
            reason=f"BRAKE: {dist:.1f}m (ratio={brake_ratio:.2f})",
        )

    def _emergency(self, dist: float, steer: float) -> ACCDecision:
        """
        Acil durum — tam fren, gaz yok.
        """
        self._state = ACCState.EMERGENCY
        self._smooth_throttle = 0.0
        self._smooth_brake = 1.0

        return ACCDecision(
            throttle=0.0,
            brake=1.0,
            steer=steer,
            state=ACCState.EMERGENCY,
            active=True,
            closest_dist_m=dist,
            reason=f"EMERGENCY: {dist:.1f}m — TAM FREN!",
        )

    # =================================================================
    #  YARDIMCI
    # =================================================================

    def _smooth(self, current: float, target: float) -> float:
        """Üstel hareketli ortalama ile yumuşatma."""
        return current + self._alpha * (target - current)
