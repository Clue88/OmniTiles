"""Synchronous wrappers around :class:`Tile` and :class:`TileFleet`.

These are thin shims that run the async SDK on a shared background event
loop. They exist so callback-driven code (notably the Viser GUI) can call
``tile.m1_extend()`` without having to manage asyncio themselves.
"""

from __future__ import annotations

import asyncio
import threading
from collections.abc import Awaitable, Callable
from concurrent.futures import Future
from typing import TypeVar

from omnitiles.fleet import TileFleet, scan as async_scan
from omnitiles.telemetry import Telemetry
from omnitiles.tile import Tile, TelemetryCallback, Unsubscribe
from omnitiles.transport import DEFAULT_TILE_NAME_PREFIX, TileInfo

T = TypeVar("T")

_loop: asyncio.AbstractEventLoop | None = None
_loop_lock = threading.Lock()


def _ensure_loop() -> asyncio.AbstractEventLoop:
    """Return the SDK's shared background event loop, starting it lazily."""
    global _loop
    with _loop_lock:
        if _loop is not None and _loop.is_running():
            return _loop
        _loop = asyncio.new_event_loop()
        started = threading.Event()

        def _runner() -> None:
            assert _loop is not None
            asyncio.set_event_loop(_loop)
            started.set()
            _loop.run_forever()

        thread = threading.Thread(target=_runner, name="omnitiles-sdk-loop", daemon=True)
        thread.start()
        started.wait()
        return _loop


def _run(coro: Awaitable[T]) -> T:
    loop = _ensure_loop()
    future: Future[T] = asyncio.run_coroutine_threadsafe(coro, loop)  # type: ignore[arg-type]
    return future.result()


def scan_sync(
    *,
    timeout: float = 5.0,
    name_prefix: str = DEFAULT_TILE_NAME_PREFIX,
) -> list[TileInfo]:
    """Synchronous version of :func:`omnitiles.scan`."""
    return _run(async_scan(timeout=timeout, name_prefix=name_prefix))


class SyncTile:
    """Blocking facade around :class:`Tile`.

    Every method delegates to the async :class:`Tile` running on the shared
    background loop. ``on_telemetry`` callbacks fire on that loop's thread;
    keep them fast and non-blocking.
    """

    def __init__(self, tile: Tile) -> None:
        self._tile = tile

    @classmethod
    def connect(cls, info: TileInfo) -> "SyncTile":
        tile = Tile(info)
        _run(tile.connect())
        return cls(tile)

    @classmethod
    def discover_one(
        cls,
        *,
        timeout: float = 5.0,
        name_prefix: str = DEFAULT_TILE_NAME_PREFIX,
    ) -> "SyncTile":
        """Scan, take the first match, and return a connected SyncTile."""
        infos = scan_sync(timeout=timeout, name_prefix=name_prefix)
        if not infos:
            raise RuntimeError(f"No tiles found with name prefix {name_prefix!r}")
        return cls.connect(infos[0])

    # ---- identity ----

    @property
    def name(self) -> str:
        return self._tile.name

    @property
    def address(self) -> str:
        return self._tile.address

    @property
    def connected(self) -> bool:
        return self._tile.connected

    def __repr__(self) -> str:
        return f"SyncTile(name={self.name!r}, connected={self.connected})"

    # ---- connection ----

    def disconnect(self) -> None:
        _run(self._tile.disconnect())

    def __enter__(self) -> "SyncTile":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # ---- commands ----

    def ping(self) -> None:
        _run(self._tile.ping())

    def m1_extend(self, speed: int = 255) -> None:
        _run(self._tile.m1_extend(speed))

    def m1_retract(self, speed: int = 255) -> None:
        _run(self._tile.m1_retract(speed))

    def m1_brake(self) -> None:
        _run(self._tile.m1_brake())

    def m1_set_position(self, position: int) -> None:
        _run(self._tile.m1_set_position(position))

    def m1_set_position_mm(self, mm: float) -> None:
        _run(self._tile.m1_set_position_mm(mm))

    def m2_extend(self, speed: int = 255) -> None:
        _run(self._tile.m2_extend(speed))

    def m2_retract(self, speed: int = 255) -> None:
        _run(self._tile.m2_retract(speed))

    def m2_brake(self) -> None:
        _run(self._tile.m2_brake())

    def m2_set_position(self, position: int) -> None:
        _run(self._tile.m2_set_position(position))

    def m2_set_position_mm(self, mm: float) -> None:
        _run(self._tile.m2_set_position_mm(mm))

    def base_velocity(self, vx: int, vy: int, omega: int) -> None:
        _run(self._tile.base_velocity(vx, vy, omega))

    def base_brake(self) -> None:
        _run(self._tile.base_brake())

    # ---- telemetry ----

    @property
    def telemetry(self) -> Telemetry | None:
        return self._tile.telemetry

    def on_telemetry(self, callback: TelemetryCallback) -> Unsubscribe:
        return self._tile.on_telemetry(callback)

    def wait_for_telemetry(self, timeout: float | None = None) -> Telemetry:
        return _run(self._tile.wait_for_telemetry(timeout))


class SyncFleet:
    """Blocking facade around :class:`TileFleet`."""

    def __init__(self, fleet: TileFleet) -> None:
        self._fleet = fleet
        self._tiles: dict[str, SyncTile] = {tile.name: SyncTile(tile) for tile in fleet}

    @classmethod
    def discover(
        cls,
        *,
        timeout: float = 5.0,
        name_prefix: str = DEFAULT_TILE_NAME_PREFIX,
    ) -> "SyncFleet":
        fleet = _run(TileFleet.discover(timeout=timeout, name_prefix=name_prefix))
        _run(fleet.connect_all())
        return cls(fleet)

    def __len__(self) -> int:
        return len(self._tiles)

    def __iter__(self):
        return iter(self._tiles.values())

    def __getitem__(self, name: str) -> SyncTile:
        return self._tiles[name]

    def __contains__(self, name: object) -> bool:
        return name in self._tiles

    @property
    def names(self) -> list[str]:
        return list(self._tiles.keys())

    def disconnect_all(self) -> None:
        _run(self._fleet.disconnect_all())

    def __enter__(self) -> "SyncFleet":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect_all()

    def broadcast(self, action: Callable[[Tile], Awaitable[None]]) -> None:
        _run(self._fleet.broadcast(action))
