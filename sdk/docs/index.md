# OmniTiles Python SDK

The OmniTiles SDK gives you a Python API for every interaction the debug GUI
supports: send motion commands to one or many tiles, subscribe to sensor
telemetry, and script higher-level behaviors on top. The
[Viser debug GUI](https://github.com/christopherliu/OmniTiles/tree/main/gui)
uses this SDK as its transport and protocol layer.

```{toctree}
:maxdepth: 2
:caption: Contents

quickstart
multi_tile
protocol
api/index
```

## Highlights

- **Async-first API** with a blocking sync wrapper for scripts and UI frameworks.
- **Multi-tile fleets** — discover every `OmniTile_*` advertiser and control
  them in parallel.
- **Typed telemetry** exposed both as a latest-snapshot property and as a
  callback subscription.
- **Protocol parity** with the Rust firmware; the binary protocol module
  mirrors `omnitiles/src/protocol/messages.rs`.

## Quick peek

```python
from omnitiles import SyncTile

tile = SyncTile.discover_one()
tile.m1_extend(speed=200)
print(tile.telemetry)
tile.disconnect()
```
