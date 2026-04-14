"""BLE transport layer wrapping :mod:`bleak`.

The SDK exposes a small :class:`Transport` protocol so alternative transports
(mocks, loopbacks, future UART) can slot in without touching :class:`Tile`.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
"""Write characteristic (host → tile)."""
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
"""Notify characteristic (tile → host)."""

DEFAULT_TILE_NAME_PREFIX = "OmniTile_"

NotifyHandler = Callable[[bytes], None]
DisconnectHandler = Callable[[], None]


class Transport(Protocol):
    """Minimal transport contract used by :class:`~omnitiles.tile.Tile`."""

    async def connect(self) -> None: ...
    async def disconnect(self) -> None: ...
    async def send(self, data: bytes) -> None: ...
    def set_notify_handler(self, handler: NotifyHandler | None) -> None: ...
    def set_disconnect_handler(self, handler: DisconnectHandler | None) -> None: ...
    @property
    def connected(self) -> bool: ...


class BleakTransport:
    """:class:`Transport` implementation on top of ``bleak``."""

    def __init__(self, address: str) -> None:
        self._address = address
        self._client: BleakClient | None = None
        self._notify_handler: NotifyHandler | None = None
        self._disconnect_handler: DisconnectHandler | None = None

    @property
    def address(self) -> str:
        return self._address

    @property
    def connected(self) -> bool:
        return self._client is not None and self._client.is_connected

    def set_notify_handler(self, handler: NotifyHandler | None) -> None:
        self._notify_handler = handler

    def set_disconnect_handler(self, handler: DisconnectHandler | None) -> None:
        self._disconnect_handler = handler

    async def connect(self) -> None:
        # Always build a fresh client on (re)connect — bleak's client is not
        # reliably reusable after the peer drops.
        self._client = BleakClient(
            self._address,
            disconnected_callback=self._on_bleak_disconnect,
        )
        await self._client.connect()
        await self._client.start_notify(NUS_TX_UUID, self._on_notify)

    def _on_bleak_disconnect(self, _client: BleakClient) -> None:
        handler = self._disconnect_handler
        if handler is not None:
            handler()

    async def disconnect(self) -> None:
        if self._client is not None and self._client.is_connected:
            try:
                await self._client.stop_notify(NUS_TX_UUID)
            except Exception:
                pass
            await self._client.disconnect()

    async def send(self, data: bytes) -> None:
        if self._client is None or not self._client.is_connected:
            raise ConnectionError("BleakTransport is not connected")
        await self._client.write_gatt_char(NUS_RX_UUID, data, response=False)

    def _on_notify(self, _sender, data: bytearray) -> None:
        handler = self._notify_handler
        if handler is not None:
            handler(bytes(data))


async def discover_tiles(
    *,
    timeout: float = 5.0,
    name_prefix: str = DEFAULT_TILE_NAME_PREFIX,
) -> list["TileInfo"]:
    """Scan for advertising OmniTile devices.

    A device matches if its advertised name starts with ``name_prefix`` or if
    it advertises the Nordic UART service UUID.
    """
    found: dict[str, TileInfo] = {}

    def _matches(device: BLEDevice, adv: AdvertisementData) -> bool:
        name = device.name or adv.local_name or ""
        if name.startswith(name_prefix):
            return True
        return NUS_SERVICE_UUID in (adv.service_uuids or [])

    devices = await BleakScanner.discover(timeout=timeout, return_adv=True)
    for address, (device, adv) in devices.items():
        if _matches(device, adv):
            name = device.name or adv.local_name or address
            found[address] = TileInfo(name=name, address=address)
    return list(found.values())


class TileInfo:
    """Lightweight descriptor returned by scanning."""

    __slots__ = ("name", "address")

    def __init__(self, name: str, address: str) -> None:
        self.name = name
        self.address = address

    def __repr__(self) -> str:
        return f"TileInfo(name={self.name!r}, address={self.address!r})"
