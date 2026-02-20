# OmniTiles GUI

Python GUI for driving the tile during development. It runs a Viser server (web UI at
http://localhost:8080) with various controls.

The GUI prefers BLE: it scans for the DWM tag and sends command packets over the Nordic UART
Service. If you pass `--port`, it also opens that serial port; when BLE isn’t connected it will send
commands over UART instead, and it always reads lines from UART and prints them to the console so
you can see STM32 debug output.

**Requirements:** Python 3.12+. Dependencies are in [pyproject.toml](pyproject.toml).

**Run:**

```bash
uv sync
uv run python main.py
```
