"""Binary protocol message IDs.

Mirrors ``omnitiles/src/protocol/messages.rs`` — keep these in sync with the
firmware. Packet format:

    [START_BYTE] [msg_id] [payload...] [checksum]

``checksum`` is the 8-bit sum of ``msg_id`` and all payload bytes.
"""

from enum import IntEnum

START_BYTE: int = 0xA5


class MessageId(IntEnum):
    """Protocol message identifiers."""

    M1_EXTEND = 0x30
    M1_RETRACT = 0x31
    M1_BRAKE = 0x32
    M1_SET_POSITION = 0x33

    M2_EXTEND = 0x40
    M2_RETRACT = 0x41
    M2_BRAKE = 0x42
    M2_SET_POSITION = 0x43

    PING = 0x50

    TELEMETRY = 0x60

    BASE_VELOCITY = 0x70
    BASE_BRAKE = 0x71
