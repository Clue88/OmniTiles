from __future__ import annotations

import time
from pathlib import Path

import viser

from omnitiles import load_anchor_positions

from advanced import UwbNoisePanel
from connection import ConnectionManager
from scene import Scene
from sidebar import Sidebar
from state import AppState

ANCHOR_CONFIG_PATH = Path(__file__).resolve().parent.parent / "configs" / "anchors.toml"


class App:
    def __init__(self, fake: bool) -> None:
        anchors = load_anchor_positions(ANCHOR_CONFIG_PATH)
        print(f"[gui] Loaded {len(anchors)} anchors from {ANCHOR_CONFIG_PATH}")

        self.state = AppState(anchors=tuple(anchors))
        self.server = viser.ViserServer(label="OmniTiles")
        self.conn = ConnectionManager(self.state, fake=fake)
        self.scene = Scene(self.server, self.state)
        self.sidebar = Sidebar(
            self.server,
            self.state,
            get_tile=self.conn.get_tile,
            list_tiles=self.conn.list_tiles,
            rescan=self.conn.rescan,
            disconnect_all=self.conn.disconnect_all,
        )
        with self.sidebar.advanced_folder:
            self.uwb_noise = UwbNoisePanel(self.server, self.state, self.conn)

    def run(self) -> None:
        self.conn.start()
        print("GUI ready at http://localhost:8080")
        try:
            while True:
                self.scene.update()
                self.sidebar.refresh()
                self.uwb_noise.refresh()
                time.sleep(1.0 / 30.0)
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            self.conn.stop()
