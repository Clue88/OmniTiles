"""Coordinated motion across every discovered tile.

Extends M1 on every tile together, then retracts — the minimal multi-tile
demo. Replace the body of ``action`` to script any other pattern.

Usage:
    uv run python sdk/examples/multi_tile_wave.py
"""

import asyncio

from omnitiles import TileFleet


async def main() -> None:
    async with await TileFleet.discover() as fleet:
        if len(fleet) == 0:
            print("No OmniTile devices found.")
            return
        print(f"Controlling {len(fleet)} tiles: {fleet.names}")

        for _ in range(3):
            await fleet.broadcast(lambda t: t.m1_extend(speed=200))
            await asyncio.sleep(1.0)
            await fleet.broadcast(lambda t: t.m1_retract(speed=200))
            await asyncio.sleep(1.0)
        await fleet.broadcast(lambda t: t.m1_brake())


if __name__ == "__main__":
    asyncio.run(main())
