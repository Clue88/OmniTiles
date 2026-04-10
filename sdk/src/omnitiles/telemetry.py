"""Telemetry data structures produced by :class:`StreamParser`."""

from dataclasses import dataclass, field


@dataclass(frozen=True, slots=True)
class ImuSample:
    """Inertial measurement sample from the on-tile IMU."""

    ax: float  #: Linear acceleration X in m/s^2.
    ay: float  #: Linear acceleration Y in m/s^2.
    az: float  #: Linear acceleration Z in m/s^2.
    gx: float  #: Angular velocity X in rad/s.
    gy: float  #: Angular velocity Y in rad/s.
    gz: float  #: Angular velocity Z in rad/s.


@dataclass(frozen=True, slots=True)
class Telemetry:
    """A parsed telemetry frame from one tile.

    Fields that weren't present in the source packet are ``None``. Sentinel
    values (``0xFFFF``) for unavailable UWB ranges and ToF readings are
    normalized to ``None`` as well.
    """

    timestamp: float
    """Host monotonic timestamp (seconds) when this frame was parsed."""

    m1_pos_adc: int
    """Raw 12-bit ADC reading for the M1 feedback potentiometer."""

    m2_pos_adc: int
    """Raw 12-bit ADC reading for the M2 feedback potentiometer."""

    m1_pos_mm: float
    """M1 position estimate in millimeters (derived from ``m1_pos_adc``)."""

    m2_pos_mm: float
    """M2 position estimate in millimeters (derived from ``m2_pos_adc``)."""

    m1_adcs: tuple[int, ...] = ()
    """Raw ADC readings for all M1 pots (up to 4). Empty if not reported."""

    m2_adcs: tuple[int, ...] = ()
    """Raw ADC readings for all M2 pots (up to 2). Empty if not reported."""

    uwb_mm: tuple[int | None, int | None, int | None] | None = None
    """Distances to anchors 0/1/2 in millimeters, or ``None`` if not reported.
    Individual entries are ``None`` when the corresponding range was invalid."""

    tof_mm: int | None = None
    """Time-of-flight sensor reading in millimeters, or ``None``."""

    imu: ImuSample | None = None
    """IMU sample, or ``None`` if the packet didn't include IMU data."""

    raw: bytes = field(default=b"", repr=False)
    """The underlying packet bytes (for debugging)."""
