"""Placeholder linear mapping between actuator mm and physical height/tilt.

We do not yet have correct kinematics from the M1 (P16 tilt) and M2 (T16 lift)
actuator positions to the physical tile height and top-platform tilt. For now
we use a proportional map across the usable stroke of each actuator. When ToF
(height) and IMU (tilt) fusion lands, replace the bodies of these functions
and everything else keeps working.
"""

from __future__ import annotations

from omnitiles import M1_CONFIG, M2_CONFIG

HEIGHT_MIN_CM = 20.0
HEIGHT_MAX_CM = 80.0

TILT_MIN_DEG = -30.0
TILT_MAX_DEG = 30.0


def _lerp(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if x1 == x0:
        return y0
    t = (x - x0) / (x1 - x0)
    t = max(0.0, min(1.0, t))
    return y0 + t * (y1 - y0)


def m2_mm_to_height_cm(m2_mm: float) -> float:
    return _lerp(
        m2_mm,
        M2_CONFIG.min_position_mm,
        M2_CONFIG.max_position_mm,
        HEIGHT_MIN_CM,
        HEIGHT_MAX_CM,
    )


def height_cm_to_m2_mm(height_cm: float) -> float:
    return _lerp(
        height_cm,
        HEIGHT_MIN_CM,
        HEIGHT_MAX_CM,
        M2_CONFIG.min_position_mm,
        M2_CONFIG.max_position_mm,
    )


def m1_mm_to_tilt_deg(m1_mm: float) -> float:
    return _lerp(
        m1_mm,
        M1_CONFIG.min_position_mm,
        M1_CONFIG.max_position_mm,
        TILT_MIN_DEG,
        TILT_MAX_DEG,
    )


def tilt_deg_to_m1_mm(tilt_deg: float) -> float:
    return _lerp(
        tilt_deg,
        TILT_MIN_DEG,
        TILT_MAX_DEG,
        M1_CONFIG.min_position_mm,
        M1_CONFIG.max_position_mm,
    )
