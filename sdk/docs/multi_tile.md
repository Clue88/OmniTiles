# Working with multiple tiles

The SDK treats multi-tile setups as first-class. Every tile is identified by
its advertised BLE name (by default, anything starting with `OmniTile_`);
give each DWM tag a unique name and the fleet API will pick them up.

## Discover and connect

```python
import asyncio
from omnitiles import TileFleet

async def main():
    async with await TileFleet.discover() as fleet:
        print(fleet.names)           # ['OmniTile_1', 'OmniTile_2', 'OmniTile_3']
        await fleet["OmniTile_2"].m1_extend(speed=255)

asyncio.run(main())
```

Using the fleet as an async context manager connects every tile on entry and
disconnects them all on exit.

## Broadcast an action in parallel

```python
await fleet.broadcast(lambda t: t.m1_set_position(128))
```

`broadcast` runs the action against every tile via `asyncio.gather`, so the
wall-clock time is roughly the time for the slowest tile, not the sum of all
tiles. Use it for coordinated layout changes and waveforms.

## Scripting a layout switch

```python
async def go_flat(fleet):
    await fleet.broadcast(lambda t: t.m1_set_position(0))
    await fleet.broadcast(lambda t: t.m2_set_position(0))

async def go_staircase(fleet):
    heights = [0, 85, 170, 255]
    for name, height in zip(sorted(fleet.names), heights):
        await fleet[name].m1_set_position(height)
```

## Sync fleet

```python
from omnitiles import SyncFleet

with SyncFleet.discover() as fleet:
    for tile in fleet:
        tile.m1_extend(speed=200)
```

`SyncFleet.broadcast` works the same way but accepts an `async` action;
everything runs on the SDK's shared background loop.
