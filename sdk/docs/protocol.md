# Binary protocol reference

The SDK speaks the same binary protocol as the Rust firmware. The canonical
definition lives in `omnitiles/src/protocol/messages.rs`; this page mirrors
it for reference.

## Packet format

```
[START_BYTE] [msg_id] [payload bytes...] [checksum]
    0xA5        u8      0, 1, or 3 bytes      u8
```

`checksum` is the 8-bit sum of `msg_id` and every payload byte (wrapping).
The SDK helper is:

```python
from omnitiles.protocol import encode, checksum, MessageId

encode(MessageId.M1_EXTEND, bytes([200]))
# b'\xa5\x30\xc8\xf8'
```

## Command IDs

| Name                | ID    | Payload     | Notes |
|---------------------|-------|-------------|-------|
| `M1_EXTEND`         | 0x30  | `u8` speed  | PWM 0–255 |
| `M1_RETRACT`        | 0x31  | `u8` speed  | |
| `M1_BRAKE`          | 0x32  | —           | |
| `M1_SET_POSITION`   | 0x33  | `u8` scaled | Target along stroke, 0–255 |
| `M2_EXTEND`         | 0x40  | `u8` speed  | |
| `M2_RETRACT`        | 0x41  | `u8` speed  | |
| `M2_BRAKE`          | 0x42  | —           | |
| `M2_SET_POSITION`   | 0x43  | `u8` scaled | |
| `PING`              | 0x50  | —           | Connectivity check |
| `TELEMETRY`         | 0x60  | —           | Response-only |
| `BASE_VELOCITY`     | 0x70  | `i8, i8, i8`| vx, vy, omega |
| `BASE_BRAKE`        | 0x71  | —           | |

## Telemetry variants

Telemetry packets are identified by total length. The parser validates the
checksum and returns a typed
[`Telemetry`](api/telemetry.rst) dataclass.

| Length | Fields |
|-------:|--------|
| 7 bytes  | `m1_pos`, `m2_pos` |
| 15 bytes | Above + `d0`, `d1`, `d2`, `d3` UWB ranges |
| 17 bytes | Above + `tof` |
| 53 bytes | Above + 6-axis IMU + 4 M1 ADCs + 2 M2 ADCs |

Invalid UWB ranges and ToF readings are encoded on-wire as `0xFFFF`. The
parser normalizes them to Python `None`.

## Extending the protocol

When adding a new message ID to the firmware:

1. Update `omnitiles/src/protocol/messages.rs`.
2. Add the corresponding entry to
   [`omnitiles.protocol.messages.MessageId`](api/protocol.rst).
3. If it's a command, add a method to `Tile` in
   `sdk/src/omnitiles/tile.py`.
4. If it's a new telemetry variant, add its byte length to
   `_TELEMETRY_LENGTHS` in `sdk/src/omnitiles/protocol/parser.py` and
   extend `_try_parse`.
5. Add a unit test in `sdk/tests/test_protocol.py`.
