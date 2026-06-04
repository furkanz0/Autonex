"""
controllers/traffic_rules_engine.py — Trafik Kuralları Öncelik Motoru

Tüm sensör girişlerini (trafik ışığı, araç tespiti) alır ve
tek bir kontrol kararı üretir. Öncelik sırası:

  1. EMERGENCY  — Çarpışma önleme (araç çok yakın)
  2. RED_LIGHT  — Kırmızı trafik ışığı
  3. BRAKING    — Kritik mesafede araç
  4. FOLLOWING  — Güvenli mesafede araç (ACC)
  5. NORMAL     — Normal şerit/waypoint kontrolü

Kullanım:
    engine = TrafficRulesEngine()
    decision = engine.decide(tl_result, vehicle_result, speed, last_steer)
    if decision.override:
        vehicle.apply_control(decision.to_carla_control())
    else:
        # Normal kontrol devam eder
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import carla

from controllers.acc_controller import AccController, ACCState, ACCDecision
from config import TARGET_SPEED_KMH, VD_SAFE_DIST_M


# =====================================================================
#  ENUM: KARAR SEBEPLERİ
# =====================================================================

class RuleReason(Enum):
    NORMAL       = auto()   # Standart kontrol
    EMERGENCY    = auto()   # Araç çok yakın — acil fren
    RED_LIGHT    = auto()   # Kırmızı ışık
    BRAKING      = auto()   # ACC frenleme
    FOLLOWING    = auto()   # ACC takip
    LANE_CHANGE_BLOCKED = auto()  # Şerit değiştirme engellendi


# =====================================================================
#  ÇIKTI VERİ YAPISI
# =====================================================================

@dataclass
class TrafficDecision:
    """Trafik kuralları motorunun ürettiği karar."""
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    override: bool = False          # True → bu değerleri direkt uygula
    allow_lane_change: bool = True  # Şerit değiştirmeye izin var mı?
    reason: RuleReason = RuleReason.NORMAL
    reason_text: str = ""           # İnsan okunabilir açıklama
    acc_decision: Optional[ACCDecision] = None

    def to_carla_control(self) -> "carla.VehicleControl":
        """CARLA VehicleControl nesnesine dönüştür."""
        return carla.VehicleControl(
            throttle=float(self.throttle),
            brake=float(self.brake),
            steer=float(self.steer),
        )

    @property
    def reason_name(self) -> str:
        return self.reason.name


# =====================================================================
#  ANA SINIF
# =====================================================================

class TrafficRulesEngine:
    """
    Tüm trafik kuralı kararlarını tek noktadan yönetir.

    Öncelik sıralaması kesin olarak uygulanır:
    Emergency > Red Light > Braking > Following > Normal
    """

    def __init__(self, target_speed_kmh: float = TARGET_SPEED_KMH):
        self._acc = AccController(target_speed_kmh=target_speed_kmh)
        self._target_kmh = target_speed_kmh

    # =================================================================
    #  PUBLIC API
    # =================================================================

    def decide(self,
               tl_result,
               vehicle_result,
               current_speed_kmh: float,
               last_steer: float = 0.0,
               lane_change_requested: bool = False) -> TrafficDecision:
        """
        Tüm girdileri değerlendirip tek kontrol kararı üret.

        Args:
            tl_result: TrafficLightResult (traffic_light_controller'dan)
            vehicle_result: VehicleDetectionResult (vehicle_detector'dan)
            current_speed_kmh: Anlık hız
            last_steer: Son direksiyon (override anında koru)
            lane_change_requested: Şerit değiştirme isteği var mı?

        Returns:
            TrafficDecision
        """
        # ── Öncelik 1: EMERGENCY (araç çok yakın) ───────────────────
        if vehicle_result.detected:
            dist = vehicle_result.closest_distance_m
            from config import VD_EMERGENCY_DIST_M
            if dist < VD_EMERGENCY_DIST_M:
                return TrafficDecision(
                    throttle=0.0,
                    brake=1.0,
                    steer=last_steer,
                    override=True,
                    allow_lane_change=False,
                    reason=RuleReason.EMERGENCY,
                    reason_text=f"EMERGENCY BRAKE: araç {dist:.1f}m",
                )

        # ── Öncelik 2: KIRMIZI IŞIK ──────────────────────────────────
        if tl_result.should_stop:
            brake_val = tl_result.brake_intensity()
            if current_speed_kmh < 1.0:
                brake_val = 0.5
            return TrafficDecision(
                throttle=0.0,
                brake=float(brake_val),
                steer=last_steer,
                override=True,
                allow_lane_change=False,
                reason=RuleReason.RED_LIGHT,
                reason_text=f"RED LIGHT STOP (brake={brake_val:.2f})",
            )

        # ── Öncelik 3 & 4: ACC (araç takibi / frenleme) ─────────────
        if vehicle_result.detected:
            acc_decision = self._acc.compute(vehicle_result, current_speed_kmh, last_steer)

            if acc_decision.active:
                # Şerit değiştirme: takip/fren modunda izin verme
                allow_lc = (
                    acc_decision.state == ACCState.FOLLOWING
                    and vehicle_result.closest_distance_m > VD_SAFE_DIST_M * 0.7
                )

                reason = (RuleReason.BRAKING
                          if acc_decision.state in (ACCState.BRAKING, ACCState.EMERGENCY)
                          else RuleReason.FOLLOWING)

                return TrafficDecision(
                    throttle=float(acc_decision.throttle),
                    brake=float(acc_decision.brake),
                    steer=last_steer,
                    override=True,
                    allow_lane_change=allow_lc,
                    reason=reason,
                    reason_text=acc_decision.reason,
                    acc_decision=acc_decision,
                )

        # ── Öncelik 5: NORMAL KONTROL ───────────────────────────────
        # Şerit değiştirme: önde araç yok, her şey normal
        allow_lane_change = True
        if vehicle_result.detected:
            # Araç var ama ACC devrede değil (çok uzak) — şerit değiştirmeye dikkat et
            dist = vehicle_result.closest_distance_m
            allow_lane_change = dist > VD_SAFE_DIST_M * 0.8

        return TrafficDecision(
            throttle=0.0,
            brake=0.0,
            steer=last_steer,
            override=False,
            allow_lane_change=allow_lane_change,
            reason=RuleReason.NORMAL,
            reason_text="NORMAL: standart şerit kontrolü",
        )

    def set_target_speed(self, speed_kmh: float):
        """Hedef hızı güncelle."""
        self._target_kmh = speed_kmh
        self._acc.set_target_speed(speed_kmh)

    @property
    def acc_state(self) -> ACCState:
        """Anlık ACC durum makinesi durumu."""
        return self._acc.state
