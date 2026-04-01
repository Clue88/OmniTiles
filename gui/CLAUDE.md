# gui — Python debug GUI

## Run

```bash
uv sync
uv run python main.py                  # BLE only
uv run python main.py --port /dev/tty.usbmodem*  # BLE + UART fallback
```

Use **uv** exclusively for package management (not pip).

## What it does

Viser web UI on localhost:8080. Connects to a DWM tag over BLE (Nordic UART Service)
and optionally over serial. Sends command packets, displays telemetry, and renders 3D
actuator positions and a 2D UWB trilateration map. Currently targets a single tile;
will need to support multiple tiles.

## Code structure

- `main.py` — single-file application
- `pyproject.toml` — dependencies (viser, bleak, pyserial, trimesh, numpy)
- `*.stl` — 3D models for actuator visualization

## Style

Formatted with `black`. Python 3.12+.

## Notes

- Protocol changes must stay in sync with `omnitiles` and `dwm_tag`.
- BLE is preferred; UART is a fallback when BLE isn't connected.
- Trilateration uses 3 anchor positions (hardcoded in `ANCHOR_POSITIONS`).
