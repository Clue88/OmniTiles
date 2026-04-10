# omnitiles-sdk

Python SDK for the OmniTiles robotics testing platform.

Lets you script anything you can do in the debug GUI: send commands, read sensor
telemetry, and coordinate multiple tiles. The OmniTiles Viser GUI is built on top
of this SDK.

## Install

```bash
uv add omnitiles-sdk                     # from a uv project
# or, from a checkout:
uv pip install -e ./sdk
```

Python 3.12+ is required.

## Quickstart (sync)

```python
from omnitiles import SyncTile, scan_sync

infos = scan_sync()                      # [TileInfo(name="OmniTile_1", ...)]
tile = SyncTile.connect(infos[0])

tile.m1_extend(speed=200)
time.sleep(1.0)
tile.m1_brake()

print(tile.telemetry)                    # latest Telemetry frame

tile.disconnect()
```

## Quickstart (async)

```python
import asyncio
from omnitiles import Tile, scan

async def main():
    [info] = await scan()
    async with Tile(info) as tile:
        await tile.m1_extend(speed=200)
        await asyncio.sleep(1.0)
        await tile.m1_brake()
        frame = await tile.wait_for_telemetry()
        print(frame)

asyncio.run(main())
```

## Multiple tiles

```python
from omnitiles import TileFleet

async def wave():
    async with await TileFleet.discover() as fleet:
        await fleet.broadcast(lambda t: t.m1_set_position(200))
```

## Documentation

Full docs build with Sphinx:

```bash
uv run --extra docs sphinx-build -b html docs docs/_build
```
