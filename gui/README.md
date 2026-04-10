# OmniTiles GUI

Viser web UI (http://localhost:8080) for driving a tile during development.
Built as a thin presentation layer on top of the [omnitiles
SDK](../sdk/README.md) — all BLE, protocol, and telemetry-parsing logic lives
in the SDK. This file only handles 3D meshes, Viser widgets, and visualization.

**Requirements:** Python 3.12+. Dependencies are in [pyproject.toml](pyproject.toml).

**Run:**

```bash
uv sync
uv run python main.py                   # connect to the first OmniTile found
uv run python main.py --tile OmniTile_2 # connect to a specific tile by name
```

The GUI currently targets a single tile at a time. To coordinate multiple
tiles without a GUI, use the SDK directly — see
[`sdk/examples/multi_tile_wave.py`](../sdk/examples/multi_tile_wave.py).
