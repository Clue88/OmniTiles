"""Multi-tile management."""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable, Iterable, Iterator

from omnitiles.tile import Tile
from omnitiles.transport import DEFAULT_TILE_NAME_PREFIX, TileInfo, discover_tiles


async def scan(
    *,
    timeout: float = 5.0,
    name_prefix: str = DEFAULT_TILE_NAME_PREFIX,
) -> list[TileInfo]:
    """Scan for OmniTile devices and return their descriptors.

    Thin wrapper around :func:`omnitiles.transport.discover_tiles` that
    re-exports with the canonical ``scan`` name.
    """
    return await discover_tiles(timeout=timeout, name_prefix=name_prefix)


class TileFleet:
    """A collection of :class:`Tile` instances addressed by name.

    :class:`TileFleet` is an async context manager: it connects every
    contained tile on ``__aenter__`` and disconnects them on ``__aexit__``.
    Use :meth:`discover` to build a fleet from a live BLE scan.
    """

    def __init__(self, tiles: Iterable[Tile]) -> None:
        self._tiles: dict[str, Tile] = {}
        for tile in tiles:
            self._tiles[tile.name] = tile

    @classmethod
    async def discover(
        cls,
        *,
        timeout: float = 5.0,
        name_prefix: str = DEFAULT_TILE_NAME_PREFIX,
    ) -> "TileFleet":
        """Scan for tiles and return a fleet wrapping all matches.

        The returned fleet is **not** yet connected — either ``await
        fleet.connect_all()`` or use it as an async context manager.
        """
        infos = await scan(timeout=timeout, name_prefix=name_prefix)
        return cls(Tile(info) for info in infos)

    # ---- collection interface ----

    def __len__(self) -> int:
        return len(self._tiles)

    def __iter__(self) -> Iterator[Tile]:
        return iter(self._tiles.values())

    def __getitem__(self, name: str) -> Tile:
        return self._tiles[name]

    def __contains__(self, name: object) -> bool:
        return name in self._tiles

    @property
    def names(self) -> list[str]:
        return list(self._tiles.keys())

    # ---- connection ----

    async def connect_all(self) -> None:
        await asyncio.gather(*(t.connect() for t in self._tiles.values()))

    async def disconnect_all(self) -> None:
        await asyncio.gather(
            *(t.disconnect() for t in self._tiles.values()),
            return_exceptions=True,
        )

    async def __aenter__(self) -> "TileFleet":
        await self.connect_all()
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        await self.disconnect_all()

    # ---- broadcast ----

    async def broadcast(
        self,
        action: Callable[[Tile], Awaitable[None]],
    ) -> None:
        """Run an async action against every tile in parallel.

        Example::

            await fleet.broadcast(lambda t: t.m1_set_position(128))
        """
        await asyncio.gather(*(action(tile) for tile in self._tiles.values()))
