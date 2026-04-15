# gui — Python debug GUI

## Run

```bash
uv sync
uv run main.py                    # connect to the first OmniTile found
uv run main.py --tile OmniTile_2  # connect to a specific tile by name
uv run main.py --all              # connect to every OmniTile discovered
```

Use **uv** exclusively for package management (not pip).

## What it does

Viser web UI on localhost:8080. Thin presentation layer on top of the
[`omnitiles-sdk`](../sdk) package: the SDK handles BLE discovery, the binary
protocol, and telemetry parsing, while this file handles Viser widgets, 3D
actuator meshes, and the UWB trilateration map.

Supports multiple tiles simultaneously via `--all`. Each connected tile gets
its own tab (controls + telemetry panels) and its own frame in the 3D scene
with actuator meshes parented under it — trilateration moves the frame, so
the whole assembly rides along with the tile's UWB position and IMU attitude.
The UWB Noise Characterization panel is shared and has a tile dropdown for
picking which tile to record from.

## Code structure

- `main.py` — Viser widgets, mesh rendering, telemetry→UI updates
- `pyproject.toml` — dependencies (`omnitiles-sdk`, viser, trimesh, numpy)
- `*.stl` — 3D models for actuator visualization

All BLE, packet encoding, checksum, and telemetry parsing logic lives in the
SDK at `../sdk/src/omnitiles/`. Don't re-implement those here.

## Style

Formatted with `black`. Python 3.12+.

## Notes

- Protocol changes happen in three places in lockstep: the Rust firmware
  (`omnitiles/src/protocol/`), the Python SDK (`sdk/src/omnitiles/protocol/`),
  and `dwm_tag`. The GUI should not need protocol edits — if you're tempted
  to change bytes here, change the SDK instead.
- Trilateration and anchor positions come from the SDK
  (`omnitiles.trilaterate`, `omnitiles.DEFAULT_ANCHOR_POSITIONS`).
- The UART fallback was removed when the SDK landed. If you need bench
  testing without a DWM tag, add a mock `Transport` to the SDK rather than
  re-adding pyserial here.
