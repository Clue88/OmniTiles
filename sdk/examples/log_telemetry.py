"""Subscribe to telemetry from a tile and print every frame.

Usage:
    uv run python sdk/examples/log_telemetry.py
"""

import time

from omnitiles import SyncTile, Telemetry


def main() -> None:
    tile = SyncTile.discover_one()
    print(f"Streaming telemetry from {tile.name}. Ctrl-C to stop.")

    def on_frame(frame: Telemetry) -> None:
        parts = [
            f"m1={frame.m1_pos_mm:6.1f}mm",
            f"m2={frame.m2_pos_mm:6.1f}mm",
        ]
        if frame.uwb_mm is not None:
            parts.append(f"uwb={frame.uwb_mm}")
        if frame.tof_mm is not None:
            parts.append(f"tof={frame.tof_mm}mm")
        if frame.imu is not None:
            parts.append(
                f"imu_a=({frame.imu.ax:+.2f},{frame.imu.ay:+.2f},{frame.imu.az:+.2f})"
            )
        print(" | ".join(parts))

    tile.on_telemetry(on_frame)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        tile.disconnect()


if __name__ == "__main__":
    main()
