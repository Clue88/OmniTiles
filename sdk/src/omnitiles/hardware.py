"""Hardware constants shared by the SDK and any presentation layer."""

from dataclasses import dataclass


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

ADC_MAX = 4095
"""Maximum 12-bit ADC reading used for raw→mm conversion."""
