"""Hardware constants shared by the SDK and any presentation layer."""

import tomllib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class ActuatorConfig:
    """Physical configuration for a linear actuator."""

    name: str
    stroke_mm: float
    buffer_bottom_mm: float
    buffer_top_mm: float
    base_stl: str
    move_stl: str

    @property
    def min_position_mm(self) -> float:
        return self.buffer_bottom_mm

    @property
    def max_position_mm(self) -> float:
        return self.stroke_mm - self.buffer_top_mm


M1_CONFIG = ActuatorConfig(
    name="P16 Linear Actuator",
    stroke_mm=150.0,
    buffer_bottom_mm=20.0,
    buffer_top_mm=35.0,
    base_stl="p16_base.stl",
    move_stl="p16_shaft.stl",
)

M2_CONFIG = ActuatorConfig(
    name="T16 Track Actuator",
    stroke_mm=100.0,
    buffer_bottom_mm=25.0,
    buffer_top_mm=15.0,
    base_stl="t16_base.stl",
    move_stl="t16_carriage.stl",
)


DEFAULT_ANCHOR_POSITIONS: tuple[tuple[float, float], ...] = (
    (0.0, 0.0),
    (3.0, 0.0),
    (0.0, 2.5),
)
"""Default UWB anchor coordinates in meters. Override per test-setup."""


def load_anchor_positions(
    path: str | Path,
) -> tuple[tuple[float, float], ...]:
    """Load UWB anchor positions from a TOML file.

    The file must contain a top-level ``positions`` array of ``[x, y]`` pairs
    in meters, ordered by ``CONFIG_ANCHOR_ID``. If the file does not exist,
    returns :data:`DEFAULT_ANCHOR_POSITIONS`.
    """
    p = Path(path)
    if not p.is_file():
        return DEFAULT_ANCHOR_POSITIONS
    with p.open("rb") as f:
        data = tomllib.load(f)
    raw = data.get("positions")
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{p}: missing or empty 'positions' array")
    result: list[tuple[float, float]] = []
    for i, entry in enumerate(raw):
        if not isinstance(entry, list) or len(entry) != 2:
            raise ValueError(f"{p}: positions[{i}] must be [x, y]")
        result.append((float(entry[0]), float(entry[1])))
    return tuple(result)


ADC_MAX = 4095
"""Maximum 12-bit ADC reading used for raw→mm conversion."""
