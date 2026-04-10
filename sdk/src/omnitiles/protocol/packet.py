"""Packet encoding for outbound commands."""

from collections.abc import Iterable

from omnitiles.protocol.messages import START_BYTE, MessageId


def checksum(msg_id: int, payload: bytes = b"") -> int:
    """Return the 8-bit protocol checksum for a message.

    The checksum is ``(msg_id + sum(payload)) & 0xFF``.
    """
    total = msg_id & 0xFF
    for byte in payload:
        total = (total + byte) & 0xFF
    return total


def encode(msg_id: int | MessageId, payload: bytes | Iterable[int] = b"") -> bytes:
    """Encode a command packet.

    Args:
        msg_id: Message identifier. Accepts :class:`MessageId` or a raw int.
        payload: Zero or more payload bytes.

    Returns:
        A ``bytes`` object ``[START_BYTE, msg_id, *payload, checksum]`` ready
        to be written to a transport.
    """
    msg_id_int = int(msg_id) & 0xFF
    payload_bytes = bytes(payload)
    csum = checksum(msg_id_int, payload_bytes)
    return bytes([START_BYTE, msg_id_int]) + payload_bytes + bytes([csum])
