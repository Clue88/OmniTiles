"""Advanced / debug panels: UWB noise characterization."""

from __future__ import annotations

import csv
import datetime as _dt
import time
from pathlib import Path
from typing import Any

import numpy as np
import viser
from viser import uplot

from omnitiles.telemetry import Telemetry

from connection import ConnectionManager
from state import AppState

ANCHOR_COLORS = ("red", "green", "blue", "orange")


class UwbNoisePanel:
    def __init__(
        self,
        server: viser.ViserServer,
        app_state: AppState,
        conn: ConnectionManager,
    ) -> None:
        self.server = server
        self.app_state = app_state
        self.conn = conn
        self.folder = server.gui.add_folder("UWB Noise Characterization")
        self._state: dict[str, Any] = {
            "recording": False,
            "start_ms": 0,
            "duration_ms": 60_000,
            "t_ms": [],
            "d": ([], [], [], []),
            "plot_handle": None,
            "tile_name": None,
        }

        with self.folder:
            self.md = server.gui.add_markdown(
                "Place the tag at a fixed position, then Record. "
                "Collects all three anchor ranges from the selected tile for "
                "the given duration."
            )
            initial_options = self._tile_options()
            self.tile_dd = server.gui.add_dropdown(
                "Tile",
                options=initial_options,
                initial_value=initial_options[0],
            )
            self.duration_s = server.gui.add_number("Duration (s)", initial_value=60.0, step=1.0)
            self.record_btn = server.gui.add_button("Record")
            self.record_btn.on_click(lambda _: self._start_recording())

        conn.add_telemetry_listener(self._on_telemetry)

    def _tile_options(self) -> tuple[str, ...]:
        names = [str(t.name) for t in self.conn.list_tiles()]
        return tuple(names) if names else ("(no tile)",)

    def refresh(self) -> None:
        options = self._tile_options()
        if tuple(self.tile_dd.options) != options:
            current = self.tile_dd.value
            self.tile_dd.options = options
            if current in options:
                self.tile_dd.value = current
            else:
                self.tile_dd.value = options[0]

    def _start_recording(self) -> None:
        if self._state["recording"]:
            return
        self._state["duration_ms"] = int(round(float(self.duration_s.value) * 1000))
        self._state["start_ms"] = int(round(time.monotonic() * 1000))
        self._state["t_ms"] = []
        self._state["d"] = ([], [], [], [])
        self._state["tile_name"] = self.tile_dd.value
        if self._state["plot_handle"] is not None:
            try:
                self._state["plot_handle"].remove()
            except Exception:
                pass
            self._state["plot_handle"] = None
        self._state["recording"] = True
        self.md.content = (
            f"Recording 0.0 / {self._state['duration_ms']/1000:.0f} s "
            f"from {self._state['tile_name']}..."
        )

    def _on_telemetry(self, name: str, frame: Telemetry) -> None:
        if not self._state["recording"]:
            return
        if self._state["tile_name"] != name:
            return
        if frame.uwb_mm is None:
            return
        now_ms = int(round(time.monotonic() * 1000))
        elapsed_ms = now_ms - self._state["start_ms"]
        self._state["t_ms"].append(elapsed_ms)
        for i in range(4):
            val = frame.uwb_mm[i]
            self._state["d"][i].append(float("nan") if val is None else float(val))
        if elapsed_ms < self._state["duration_ms"]:
            self.md.content = (
                f"Recording {elapsed_ms/1000:.1f} / "
                f"{self._state['duration_ms']/1000:.0f} s "
                f"from {self._state['tile_name']}... "
                f"({len(self._state['t_ms'])} samples)"
            )
            return

        self._state["recording"] = False
        self._finalize()

    def _finalize(self) -> None:
        t_s = np.array(self._state["t_ms"], dtype=float) / 1000.0
        d_arrs = [np.array(self._state["d"][i], dtype=float) for i in range(4)]

        n_samples = len(t_s)
        duration_s = float(t_s[-1]) if n_samples > 0 else 0.0
        sample_rate = n_samples / duration_s if duration_s > 0 else 0.0

        log_dir = Path(__file__).resolve().parent / "logs"
        log_dir.mkdir(exist_ok=True)
        ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = str(self._state["tile_name"]).replace(" ", "_")
        csv_path = log_dir / f"uwb_noise_{safe_name}_{ts}.csv"
        with csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow([f"# recorded_at={_dt.datetime.now().isoformat()}"])
            w.writerow([f"# tile={self._state['tile_name']}"])
            w.writerow([f"# anchors_m={[list(a) for a in self.app_state.anchors]}"])
            w.writerow([f"# n_samples={n_samples}"])
            w.writerow([f"# duration_s={duration_s:.3f}"])
            w.writerow([f"# sample_rate_hz={sample_rate:.2f}"])
            w.writerow(["t_s", "d0_mm", "d1_mm", "d2_mm", "d3_mm"])
            for k in range(n_samples):
                row = [f"{t_s[k]:.3f}"]
                for i in range(4):
                    v = d_arrs[i][k]
                    row.append("" if not np.isfinite(v) else f"{int(v)}")
                w.writerow(row)

        lines = [
            f"**UWB noise over {duration_s:.1f} s "
            f"({n_samples} samples, {sample_rate:.1f} Hz, "
            f"tile {self._state['tile_name']})**"
        ]
        for i, d in enumerate(d_arrs):
            finite = d[np.isfinite(d)]
            if finite.size == 0:
                lines.append(f"- **A{i}:** no samples")
                continue
            lines.append(
                f"- **A{i}:** mean {finite.mean():.1f} mm, "
                f"median {float(np.median(finite)):.1f} mm, "
                f"std {finite.std(ddof=1) if finite.size > 1 else 0.0:.1f} mm, "
                f"range {finite.min():.0f}–{finite.max():.0f} mm, "
                f"n={finite.size}"
            )
        lines.append(f"Saved: `{csv_path}`")
        self.md.content = "  \n".join(lines)

        series = (
            uplot.Series(label="t (s)"),
            uplot.Series(label="A0 (mm)", stroke=ANCHOR_COLORS[0]),
            uplot.Series(label="A1 (mm)", stroke=ANCHOR_COLORS[1]),
            uplot.Series(label="A2 (mm)", stroke=ANCHOR_COLORS[2]),
            uplot.Series(label="A3 (mm)", stroke=ANCHOR_COLORS[3]),
        )
        with self.folder:
            self._state["plot_handle"] = self.server.gui.add_uplot(
                data=(t_s, d_arrs[0], d_arrs[1], d_arrs[2], d_arrs[3]),
                series=series,
                title=f"UWB ranges over time ({self._state['tile_name']})",
                aspect=2.0,
            )
