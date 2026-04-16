"""Tests for the packet encoder and stream parser."""

import struct

from omnitiles.hardware import ADC_MAX, M1_CONFIG, M2_CONFIG
from omnitiles.protocol import MessageId, StreamParser, encode
from omnitiles.protocol.packet import checksum


def test_encode_no_payload():
    packet = encode(MessageId.PING)
    assert packet == bytes([0xA5, 0x50, 0x50])


def test_encode_single_byte_payload():
    packet = encode(MessageId.M1_EXTEND, bytes([200]))
    expected_csum = (0x30 + 200) & 0xFF
    assert packet == bytes([0xA5, 0x30, 200, expected_csum])


def test_encode_multi_byte_payload():
    payload = struct.pack("<bbb", 10, -5, 20)
    packet = encode(MessageId.BASE_VELOCITY, payload)
    expected_csum = checksum(0x70, payload)
    assert packet[:2] == bytes([0xA5, 0x70])
    assert packet[2:5] == payload
    assert packet[5] == expected_csum


def _telemetry_packet(body: bytes) -> bytes:
    """Wrap [msg_id, body] with start byte + checksum."""
    csum = checksum(MessageId.TELEMETRY, body)
    return bytes([0xA5, MessageId.TELEMETRY]) + body + bytes([csum])


def test_parse_minimal_packet():
    body = struct.pack("<HH", 2000, 1000)
    packet = _telemetry_packet(body)
    assert len(packet) == 7

    parser = StreamParser()
    [frame] = parser.feed(packet)
    assert frame.m1_pos_adc == 2000
    assert frame.m2_pos_adc == 1000
    assert frame.m1_pos_mm == (2000 / ADC_MAX) * M1_CONFIG.stroke_mm
    assert frame.m2_pos_mm == (1000 / ADC_MAX) * M2_CONFIG.stroke_mm
    assert frame.uwb_mm is None
    assert frame.tof_mm is None
    assert frame.imu is None


def test_parse_uwb_packet():
    body = struct.pack("<HHHHHH", 2000, 1000, 1500, 2500, 0xFFFF, 3000)
    packet = _telemetry_packet(body)
    assert len(packet) == 15

    parser = StreamParser()
    [frame] = parser.feed(packet)
    assert frame.uwb_mm == (1500, 2500, None, 3000)
    assert frame.tof_mm is None


def test_parse_uwb_tof_packet():
    body = struct.pack("<HHHHHHH", 2000, 1000, 1500, 2500, 3500, 4500, 321)
    packet = _telemetry_packet(body)
    assert len(packet) == 17

    parser = StreamParser()
    [frame] = parser.feed(packet)
    assert frame.uwb_mm == (1500, 2500, 3500, 4500)
    assert frame.tof_mm == 321


def test_parse_full_packet_with_imu_and_adcs():
    body = struct.pack(
        "<HHHHHHH6f4H2H",
        2000,
        1000,
        1500,
        2500,
        3500,
        4500,
        321,
        0.1,
        0.2,
        9.8,
        0.01,
        -0.02,
        0.03,
        2001,
        2002,
        2003,
        2004,
        1001,
        1002,
    )
    packet = _telemetry_packet(body)
    assert len(packet) == 53

    parser = StreamParser()
    [frame] = parser.feed(packet)
    assert frame.uwb_mm == (1500, 2500, 3500, 4500)
    assert frame.tof_mm == 321
    assert frame.imu is not None
    assert abs(frame.imu.az - 9.8) < 1e-6
    assert frame.m1_adcs == (2001, 2002, 2003, 2004)
    assert frame.m2_adcs == (1001, 1002)


def test_parse_rejects_bad_checksum():
    body = struct.pack("<HH", 2000, 1000)
    packet = bytearray(_telemetry_packet(body))
    packet[-1] ^= 0xFF  # corrupt checksum

    parser = StreamParser()
    assert parser.feed(bytes(packet)) == []


def test_parse_handles_partial_bytes():
    body = struct.pack("<HH", 1234, 5678)
    packet = _telemetry_packet(body)

    parser = StreamParser()
    assert parser.feed(packet[:3]) == []
    assert parser.feed(packet[3:6]) == []
    [frame] = parser.feed(packet[6:])
    assert frame.m1_pos_adc == 1234


def test_parse_back_to_back_packets():
    body_a = struct.pack("<HH", 100, 200)
    body_b = struct.pack("<HH", 300, 400)
    stream = _telemetry_packet(body_a) + _telemetry_packet(body_b)

    parser = StreamParser()
    frames = parser.feed(stream)
    assert len(frames) == 2
    assert frames[0].m1_pos_adc == 100
    assert frames[1].m1_pos_adc == 300


def test_parse_skips_garbage_before_start_byte():
    body = struct.pack("<HH", 111, 222)
    packet = _telemetry_packet(body)
    stream = b"\x00\x01\x02" + packet

    parser = StreamParser()
    [frame] = parser.feed(stream)
    assert frame.m1_pos_adc == 111


def test_parser_invalid_sentinels_become_none():
    body = struct.pack("<HHHHHH", 0, 0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF)
    packet = _telemetry_packet(body)

    parser = StreamParser()
    [frame] = parser.feed(packet)
    assert frame.uwb_mm == (None, None, None, None)
