"""Tile connection + telemetry wiring.

Supports both fake and real modes so the sidebar and scene don't need to know
which they're talking to.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable

from omnitiles import SyncTile, scan_sync, trilaterate
from omnitiles.telemetry import Telemetry

from fake_tile import FakeTile, make_fake_tiles
from state import AppState, TileState

ANCHOR_HEIGHT_M = 0.75
TAG_HEIGHT_M = 0.75


class ConnectionManager:
    def __init__(self, app_state: AppState, fake: bool) -> None:
        self.app_state = app_state
        self.fake = fake
        self._tiles: dict[str, SyncTile | FakeTile] = {}
        self._unsubs: dict[str, Callable] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._scan_thread: threading.Thread | None = None
        self._extra_listeners: list = []

    def add_telemetry_listener(self, cb) -> None:
        """Register ``cb(name, frame)`` to fire on every telemetry packet."""
        self._extra_listeners.append(cb)

    def start(self) -> None:
        if self.fake:
            self._start_fake()
            return
        self._scan_thread = threading.Thread(target=self._scan_loop, name="gui-scan", daemon=True)
        self._scan_thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.disconnect_all()

    def list_tiles(self) -> list:
        with self._lock:
            return list(self._tiles.values())

    def get_tile(self, name: str):
        with self._lock:
            return self._tiles.get(name)

    def rescan(self) -> None:
        if self.fake:
            return
        threading.Thread(target=self._do_scan, daemon=True).start()

    def disconnect_all(self) -> None:
        with self._lock:
            tiles = list(self._tiles.values())
            unsubs = list(self._unsubs.values())
            self._tiles.clear()
            self._unsubs.clear()
        for unsub in unsubs:
            try:
                unsub()
            except Exception:
                pass
        for t in tiles:
            try:
                t.disconnect()
            except Exception as e:
                name = getattr(t, "name", "?")
                print(f"[conn] Disconnect {name} failed: {e}")
        for name in list(self.app_state.tiles.keys()):
            self.app_state.tiles[name].connected = False

    # ----- fake -----

    def _start_fake(self) -> None:
        for ft in make_fake_tiles(self.app_state.anchors):
            self._register(ft)

    # ----- real -----

    def _scan_loop(self) -> None:
        while not self._stop.is_set():
            have_any = any(getattr(t, "connected", False) for t in self.list_tiles())
            if not have_any:
                self._do_scan()
            # Keep state.connected in sync with transport status.
            self._poll_connection_status()
            self._stop.wait(2.0)

    def _do_scan(self) -> None:
        self.app_state.scanning = True
        try:
            try:
                infos = scan_sync(timeout=5.0)
            except Exception as e:
                print(f"[conn] Scan failed: {e}")
                return
            for info in infos:
                with self._lock:
                    if info.name in self._tiles:
                        continue
                print(f"[conn] Connecting to {info.name} ({info.address})")
                try:
                    tile = SyncTile.connect(info)
                except Exception as e:
                    print(f"[conn] Connect {info.name} failed: {e}")
                    continue
                self._register(tile)
        finally:
            self.app_state.scanning = False

    def _poll_connection_status(self) -> None:
        for name, tile in list(self._tiles.items()):
            st = self.app_state.tiles.get(name)
            if st is None:
                continue
            st.connected = bool(getattr(tile, "connected", False))

    # ----- shared -----

    def _register(self, tile) -> None:
        name = str(tile.name)
        with self._lock:
            self._tiles[name] = tile
        if name not in self.app_state.tiles:
            self.app_state.tiles[name] = TileState(name=name)
        self.app_state.tiles[name].connected = bool(getattr(tile, "connected", True))

        def _cb(frame: Telemetry, _name=name) -> None:
            self._on_telemetry(_name, frame)

        unsub = tile.on_telemetry(_cb)
        with self._lock:
            self._unsubs[name] = unsub

    def _on_telemetry(self, name: str, frame: Telemetry) -> None:
        st = self.app_state.tiles.get(name)
        if st is None:
            return
        st.last_telemetry_ts = time.monotonic()
        st.connected = True
        st.m1_mm = frame.m1_pos_mm
        st.m2_mm = frame.m2_pos_mm
        st.tof_mm = frame.tof_mm

        if frame.uwb_mm is not None and all(v is not None for v in frame.uwb_mm):
            z_offset = ANCHOR_HEIGHT_M - TAG_HEIGHT_M
            pos = trilaterate(frame.uwb_mm, self.app_state.anchors, z_offset_m=z_offset)
            if pos is not None:
                st.xy_m = (float(pos[0]), float(pos[1]))

        if frame.imu is not None:
            imu = frame.imu
            norm = math.sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az)
            if norm > 1e-3:
                st.imu_roll_deg = math.degrees(math.atan2(imu.ay, imu.az))
                st.imu_pitch_deg = math.degrees(
                    math.atan2(-imu.ax, math.sqrt(imu.ay * imu.ay + imu.az * imu.az))
                )

        for cb in list(self._extra_listeners):
            try:
                cb(name, frame)
            except Exception as e:
                print(f"[conn] Extra listener error: {e}")
