from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class TileState:
    name: str
    connected: bool = False
    last_telemetry_ts: float | None = None

    # Commanded setpoints (what the user asked for via the sliders).
    cmd_height_cm: float = 20.0
    cmd_tilt_deg: float = 0.0

    # Current telemetry-derived values.
    m1_mm: float | None = None
    m2_mm: float | None = None
    m1_pos_adc: int | None = None
    m2_pos_adc: int | None = None
    m1_adcs: tuple[int, ...] = ()
    m2_adcs: tuple[int, ...] = ()
    xy_m: tuple[float, float] | None = None
    imu_roll_deg: float | None = None
    imu_pitch_deg: float | None = None
    tof_mm: int | None = None


@dataclass
class AppState:
    anchors: tuple[tuple[float, float], ...]
    tiles: dict[str, TileState] = field(default_factory=dict)
    scanning: bool = False
