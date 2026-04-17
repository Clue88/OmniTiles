"""Mapping between actuator mm / ToF readings and physical height/tilt.

The T16 lift actuator is reversed: retracting (lower m2_mm) raises the tile.
When a ToF reading is available it gives a direct height measurement and is
preferred over the M2 actuator mapping.

ToF sensor geometry (all values in cm):
  - Sensor is mounted 15 cm above ground on the fixed base.
  - Sensor points up at a reflector plate on the moving platform.
  - The plate is 17 cm below the top surface of the tile.
  - tile_top_height = SENSOR_HEIGHT + tof_distance + PLATE_OFFSET
"""

from __future__ import annotations

from omnitiles import M1_CONFIG, M2_CONFIG

HEIGHT_MIN_CM = 20.0
HEIGHT_MAX_CM = 80.0

TOF_SENSOR_HEIGHT_CM = 15.0
TOF_PLATE_OFFSET_CM = 17.0

# Calibrated from IMU measurements (least-squares fit):
#   +22.2 deg at 19.7 mm,  0 deg at 66.5 mm,  -24.9 deg at 115.2 mm
_TILT_CAL = ((19.7, 22.2), (66.5, 0.0), (115.2, -24.9))
_n = len(_TILT_CAL)
_mx = sum(p[0] for p in _TILT_CAL) / _n
_my = sum(p[1] for p in _TILT_CAL) / _n
_TILT_SLOPE = sum((x - _mx) * (y - _my) for x, y in _TILT_CAL) / sum(
    (x - _mx) ** 2 for x, y in _TILT_CAL
)
_TILT_INTERCEPT = _my - _TILT_SLOPE * _mx

TILT_MIN_DEG = round(_TILT_SLOPE * M1_CONFIG.max_position_mm + _TILT_INTERCEPT, 1)
TILT_MAX_DEG = round(_TILT_SLOPE * M1_CONFIG.min_position_mm + _TILT_INTERCEPT, 1)


def _lerp(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if x1 == x0:
        return y0
    t = (x - x0) / (x1 - x0)
    t = max(0.0, min(1.0, t))
    return y0 + t * (y1 - y0)


def m2_mm_to_height_cm(m2_mm: float) -> float:
    # Reversed: retracting T16 (lower mm) raises the tile.
    return _lerp(
        m2_mm,
        M2_CONFIG.min_position_mm,
        M2_CONFIG.max_position_mm,
        HEIGHT_MAX_CM,
        HEIGHT_MIN_CM,
    )


def height_cm_to_m2_mm(height_cm: float) -> float:
    return _lerp(
        height_cm,
        HEIGHT_MIN_CM,
        HEIGHT_MAX_CM,
        M2_CONFIG.max_position_mm,
        M2_CONFIG.min_position_mm,
    )


def tof_mm_to_height_cm(tof_mm: float) -> float:
    return TOF_SENSOR_HEIGHT_CM + (tof_mm / 10.0) + TOF_PLATE_OFFSET_CM


def m1_mm_to_tilt_deg(m1_mm: float) -> float:
    return _TILT_SLOPE * m1_mm + _TILT_INTERCEPT


def tilt_deg_to_m1_mm(tilt_deg: float) -> float:
    return (tilt_deg - _TILT_INTERCEPT) / _TILT_SLOPE
