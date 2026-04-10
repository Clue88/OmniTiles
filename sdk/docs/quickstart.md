# Quickstart

## Install

From a checkout of the OmniTiles repository:

```bash
cd sdk
uv sync
```

From another `uv` project:

```bash
uv add omnitiles-sdk
```

Python 3.12 or newer is required.

## Connect to one tile (blocking)

```python
import time
from omnitiles import SyncTile

tile = SyncTile.discover_one()
print(f"Connected to {tile.name}")

tile.m1_extend(speed=200)
time.sleep(1.0)
tile.m1_brake()

# Latest sensor snapshot
frame = tile.telemetry
if frame is not None:
    print("M1 position:", frame.m1_pos_mm, "mm")

tile.disconnect()
```

## Connect to one tile (async)

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

## Subscribe to telemetry

```python
from omnitiles import SyncTile, Telemetry

tile = SyncTile.discover_one()

def on_frame(frame: Telemetry) -> None:
    print(frame.m1_pos_mm, frame.m2_pos_mm)

unsubscribe = tile.on_telemetry(on_frame)
# ... later:
unsubscribe()
```

Callbacks run on the SDK's background event loop, so keep them fast and
non-blocking. For UI frameworks, marshal back to the UI thread inside the
callback.
