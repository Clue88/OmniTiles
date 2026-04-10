"""Sync example: extend and retract M1 in a loop.

Usage:
    uv run python sdk/examples/blink_m1.py
"""

import time

from omnitiles import SyncTile


def main() -> None:
    tile = SyncTile.discover_one()
    print(f"Connected to {tile.name} ({tile.address})")

    try:
        for _ in range(3):
            print("extend")
            tile.m1_extend(speed=200)
            time.sleep(1.0)
            print("retract")
            tile.m1_retract(speed=200)
            time.sleep(1.0)
        tile.m1_brake()
    finally:
        tile.disconnect()


if __name__ == "__main__":
    main()
