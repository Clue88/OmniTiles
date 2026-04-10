"""Stateful parser for inbound telemetry frames."""

import struct
import time
from collections.abc import Iterator

from omnitiles.hardware import ADC_MAX, M1_CONFIG, M2_CONFIG
from omnitiles.protocol.messages import START_BYTE, MessageId
from omnitiles.telemetry import ImuSample, Telemetry

# Known telemetry packet lengths (bytes on the wire, including start byte and
# checksum). Each variant is distinguished only by length; add new entries
# here when the firmware grows the packet.
_TELEMETRY_LENGTHS = (7, 13, 15, 51)
_MAX_TELEMETRY_LEN = max(_TELEMETRY_LENGTHS)


class StreamParser:
    """Feed BLE notification bytes in, get :class:`Telemetry` frames out.

    The parser maintains an internal byte buffer and hunts for ``0xA5 0x60``
    packets. Because telemetry packets don't carry an explicit length byte,
    the parser identifies variants by total length and validates the
    checksum. Invalid/corrupt frames are silently skipped.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes | bytearray) -> list[Telemetry]:
        """Append ``data`` to the internal buffer and return any frames
        completed by the new bytes."""
        self._buf.extend(data)
        return list(self._drain())

    def _drain(self) -> Iterator[Telemetry]:
        while True:
            frame = self._try_consume_frame()
            if frame is None:
                return
            yield frame

    def _try_consume_frame(self) -> Telemetry | None:
        buf = self._buf

        while buf and buf[0] != START_BYTE:
            del buf[0]

        if len(buf) < 2:
            return None

        if buf[1] != MessageId.TELEMETRY:
            del buf[0]
            return None

        # Try the longest length we have enough bytes for first, so that
        # short prefixes of a longer packet don't cause a false positive on
        # a rare 1-in-256 checksum collision. If nothing validates at the
        # longest length but we might still be waiting for the tail of a
        # bigger packet, return None and wait for more bytes.
        if len(buf) < min(_TELEMETRY_LENGTHS):
            return None

        for length in sorted(_TELEMETRY_LENGTHS, reverse=True):
            if len(buf) < length:
                continue
            frame = _try_parse(bytes(buf[:length]))
            if frame is not None:
                del buf[:length]
                return frame

        if len(buf) < _MAX_TELEMETRY_LEN:
            return None

        # We have enough bytes for the largest possible packet and none
        # validated — this start byte is junk. Advance.
        del buf[0]
        return None


def _checksum_ok(packet: bytes) -> bool:
    total = 0
    for byte in packet[1:-1]:
        total = (total + byte) & 0xFF
    return packet[-1] == total


def _try_parse(packet: bytes) -> Telemetry | None:
    if not _checksum_ok(packet):
        return None

    length = len(packet)
    m1_pos_adc, m2_pos_adc = struct.unpack_from("<HH", packet, 2)
    m1_pos_mm = (m1_pos_adc / ADC_MAX) * M1_CONFIG.stroke_mm
    m2_pos_mm = (m2_pos_adc / ADC_MAX) * M2_CONFIG.stroke_mm

    uwb_mm: tuple[int | None, int | None, int | None] | None = None
    tof_mm: int | None = None
    imu: ImuSample | None = None
    m1_adcs: tuple[int, ...] = ()
    m2_adcs: tuple[int, ...] = ()

    if length >= 13:
        d0, d1, d2 = struct.unpack_from("<HHH", packet, 6)
        uwb_mm = (
            None if d0 == 0xFFFF else d0,
            None if d1 == 0xFFFF else d1,
            None if d2 == 0xFFFF else d2,
        )

    if length >= 15:
        (tof_raw,) = struct.unpack_from("<H", packet, 12)
        tof_mm = None if tof_raw == 0xFFFF else tof_raw

    if length == 51:
        imu_vals = struct.unpack_from("<6f", packet, 14)
        imu = ImuSample(*imu_vals)
        m1_adcs = tuple(struct.unpack_from("<4H", packet, 38))
        m2_adcs = tuple(struct.unpack_from("<2H", packet, 46))

    return Telemetry(
        timestamp=time.monotonic(),
        m1_pos_adc=m1_pos_adc,
        m2_pos_adc=m2_pos_adc,
        m1_pos_mm=m1_pos_mm,
        m2_pos_mm=m2_pos_mm,
        m1_adcs=m1_adcs,
        m2_adcs=m2_adcs,
        uwb_mm=uwb_mm,
        tof_mm=tof_mm,
        imu=imu,
        raw=packet,
    )
