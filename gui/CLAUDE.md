# gui — Python debug GUI

## Run

```bash
uv sync
uv run main.py          # scan BLE and auto-connect to any OmniTiles
uv run main.py --fake   # no hardware; run with simulated tiles
```

Use **uv** exclusively for package management (not pip).

## What it does

Viser web UI on localhost:8080. Presentation layer on top of the
[`omnitiles-sdk`](../sdk) package: the SDK handles BLE discovery, the binary
protocol, and telemetry parsing; the GUI renders a 3D scene and sidebar for
commanding tiles and monitoring telemetry.

The 3D canvas shows each anchor and each tile as a 27×27 cm wedge whose
height (20–80 cm) and tilt (±30°) reflect the real actuator positions. The
sidebar has per-tile sliders for commanded height/tilt, a raw motor debug
section, hardcoded preset configurations (flat, ramp, stairs, …), a d-pad
for the mobile base (tile 1 only), and a collapsed Advanced folder with the
UWB noise characterization tool. Connection status is monitored live and
the scan loop auto-reconnects while no tiles are connected.

The placeholder linear mapping between actuator mm and physical height/tilt
lives in `mapping.py` — swap it out when real kinematics (ToF + IMU fusion)
land.

## Code structure

- `main.py` — entry point, argparse, runs `App`
- `app.py` — glues viser server, connection manager, scene, sidebar, advanced
- `state.py` — `AppState` / `TileState` dataclasses
- `mapping.py` — placeholder M1↔tilt / M2↔height conversions
- `connection.py` — scan + telemetry wiring, shared between fake and real tiles
- `scene.py` — 3D scene: anchors, tile wedge prisms (built with trimesh)
- `sidebar.py` — Viser GUI widgets (connection, presets, per-tile, base)
- `presets.py` — hardcoded built-in presets
- `advanced.py` — UWB noise characterization panel
- `fake_tile.py` — in-process simulated tile for `--fake`
- `pyproject.toml` — dependencies (`omnitiles-sdk`, viser, trimesh, numpy)

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
