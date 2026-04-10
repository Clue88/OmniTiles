"""High-level asynchronous API for a single OmniTile."""

from __future__ import annotations

import asyncio
import struct
from collections.abc import Callable
from typing import TYPE_CHECKING

from omnitiles.hardware import M1_CONFIG, M2_CONFIG, ActuatorConfig
from omnitiles.protocol import MessageId, StreamParser, encode
from omnitiles.telemetry import Telemetry
from omnitiles.transport import BleakTransport, Transport

if TYPE_CHECKING:
    from omnitiles.transport import TileInfo

TelemetryCallback = Callable[[Telemetry], None]
Unsubscribe = Callable[[], None]


class Tile:
    """Async interface to a single OmniTile over BLE.

    A :class:`Tile` owns a :class:`Transport` and a :class:`StreamParser`.
    Every command method returns once the packet has been handed to the
    transport — the protocol does not ACK, so a successful ``await`` means
    "sent", not "executed". Subscribe to telemetry to observe effects.

    Tiles are async context managers::

        async with Tile(info) as tile:
            await tile.m1_extend()
    """

    def __init__(
        self,
        info: "TileInfo",
        *,
        transport: Transport | None = None,
    ) -> None:
        self._info = info
        self._transport: Transport = transport or BleakTransport(info.address)
        self._parser = StreamParser()
        self._latest: Telemetry | None = None
        self._callbacks: list[TelemetryCallback] = []
        self._new_frame_event = asyncio.Event()
        self._transport.set_notify_handler(self._on_bytes)
        self._transport.set_disconnect_handler(self._on_transport_disconnect)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._reconnect_task: asyncio.Task[None] | None = None
        self._user_disconnected = False

    # ---- identity ----

    @property
    def name(self) -> str:
        return self._info.name

    @property
    def address(self) -> str:
        return self._info.address

    @property
    def connected(self) -> bool:
        return self._transport.connected

    def __repr__(self) -> str:
        return f"Tile(name={self.name!r}, connected={self.connected})"

    # ---- connection ----

    async def connect(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._user_disconnected = False
        await self._transport.connect()

    async def disconnect(self) -> None:
        self._user_disconnected = True
        if self._reconnect_task is not None and not self._reconnect_task.done():
            self._reconnect_task.cancel()
        await self._transport.disconnect()

    def _on_transport_disconnect(self) -> None:
        """Called from bleak's disconnect callback (runs on the SDK loop)."""
        if self._user_disconnected:
            return
        if self._loop is None or self._loop.is_closed():
            return
        if self._reconnect_task is not None and not self._reconnect_task.done():
            return
        self._reconnect_task = self._loop.create_task(self._reconnect_loop())

    async def _reconnect_loop(self) -> None:
        """Retry :meth:`Transport.connect` with bounded exponential backoff."""
        delay = 1.0
        max_delay = 10.0
        while not self._user_disconnected:
            try:
                await self._transport.connect()
                return
            except asyncio.CancelledError:
                raise
            except Exception:
                await asyncio.sleep(delay)
                delay = min(delay * 2.0, max_delay)

    async def __aenter__(self) -> "Tile":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        await self.disconnect()

    # ---- commands ----

    async def ping(self) -> None:
        await self._send(MessageId.PING)

    async def m1_extend(self, speed: int = 255) -> None:
        """Drive M1 in the extend direction at the given PWM speed (0-255)."""
        await self._send(MessageId.M1_EXTEND, _u8(speed))

    async def m1_retract(self, speed: int = 255) -> None:
        await self._send(MessageId.M1_RETRACT, _u8(speed))

    async def m1_brake(self) -> None:
        await self._send(MessageId.M1_BRAKE)

    async def m1_set_position(self, position: int) -> None:
        """Command a closed-loop M1 target, scaled 0-255 over the stroke."""
        await self._send(MessageId.M1_SET_POSITION, _u8(position))

    async def m1_set_position_mm(self, mm: float) -> None:
        """Convenience: set M1 target in millimeters along the stroke."""
        await self.m1_set_position(_mm_to_scaled(mm, M1_CONFIG))

    async def m2_extend(self, speed: int = 255) -> None:
        await self._send(MessageId.M2_EXTEND, _u8(speed))

    async def m2_retract(self, speed: int = 255) -> None:
        await self._send(MessageId.M2_RETRACT, _u8(speed))

    async def m2_brake(self) -> None:
        await self._send(MessageId.M2_BRAKE)

    async def m2_set_position(self, position: int) -> None:
        await self._send(MessageId.M2_SET_POSITION, _u8(position))

    async def m2_set_position_mm(self, mm: float) -> None:
        await self.m2_set_position(_mm_to_scaled(mm, M2_CONFIG))

    async def base_velocity(self, vx: int, vy: int, omega: int) -> None:
        """Command open-loop mobile-base velocity. Each component is int8."""
        payload = struct.pack("<bbb", _i8(vx), _i8(vy), _i8(omega))
        await self._send(MessageId.BASE_VELOCITY, payload)

    async def base_brake(self) -> None:
        await self._send(MessageId.BASE_BRAKE)

    # ---- telemetry ----

    @property
    def telemetry(self) -> Telemetry | None:
        """Most recently received telemetry frame, or ``None``."""
        return self._latest

    def on_telemetry(self, callback: TelemetryCallback) -> Unsubscribe:
        """Register ``callback`` to run on every incoming telemetry frame.

        Returns a zero-argument function that removes the subscription.
        Callbacks fire on whatever thread/loop the transport delivers
        notifications on, so they must not block.
        """
        self._callbacks.append(callback)

        def _unsubscribe() -> None:
            try:
                self._callbacks.remove(callback)
            except ValueError:
                pass

        return _unsubscribe

    async def wait_for_telemetry(self, timeout: float | None = None) -> Telemetry:
        """Await the next telemetry frame and return it."""
        self._new_frame_event.clear()
        await asyncio.wait_for(self._new_frame_event.wait(), timeout=timeout)
        assert self._latest is not None
        return self._latest

    # ---- internals ----

    async def _send(self, msg_id: MessageId, payload: bytes = b"") -> None:
        await self._transport.send(encode(msg_id, payload))

    def _on_bytes(self, data: bytes) -> None:
        for frame in self._parser.feed(data):
            self._latest = frame
            self._new_frame_event.set()
            for cb in list(self._callbacks):
                try:
                    cb(frame)
                except Exception:
                    pass


def _u8(value: int) -> bytes:
    return bytes([max(0, min(255, int(value)))])


def _i8(value: int) -> int:
    return max(-128, min(127, int(value)))


def _mm_to_scaled(mm: float, config: ActuatorConfig) -> int:
    """Map a millimeter target to the firmware's 0-255 set-position scale."""
    frac = mm / config.stroke_mm
    return max(0, min(255, int(round(frac * 255))))
