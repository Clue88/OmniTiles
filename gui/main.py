import argparse
import csv
import datetime as _dt
import math
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
import trimesh
import viser
from trimesh.visual import to_rgba
from trimesh.visual.color import ColorVisuals
from viser import uplot

from omnitiles import (
    M1_CONFIG,
    M2_CONFIG,
    SyncTile,
    Telemetry,
    load_anchor_positions,
    scan_sync,
    trilaterate,
)

ANCHOR_CONFIG_PATH = Path(__file__).resolve().parent.parent / "configs" / "anchors.toml"

ANCHOR_HEIGHT_M = 0.75
TAG_HEIGHT_M = 0.75

# Per-tile marker colours so the tiles are visually distinguishable in the scene.
TILE_COLORS = [
    (0, 200, 255),
    (255, 80, 200),
    (120, 255, 80),
]


def _select_tiles(name: Optional[str], connect_all: bool) -> list[SyncTile]:
    infos = scan_sync()
    if not infos:
        print("[SDK] No OmniTile devices found. Starting GUI with no tiles.")
        return []

    if name is not None:
        infos = [i for i in infos if i.name == name]
        if not infos:
            print(f"[SDK] Tile {name!r} not found.")
            return []
    elif not connect_all:
        infos = [infos[0]]

    tiles: list[SyncTile] = []
    for info in infos:
        print(f"[SDK] Connecting to {info.name} ({info.address})...")
        try:
            tiles.append(SyncTile.connect(info))
        except Exception as e:
            print(f"[SDK] Failed to connect to {info.name}: {e}")
    return tiles


def load_mesh(server, name, filename, color, pos, rotation=None):
    if not os.path.exists(filename):
        return server.scene.add_box(name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos)
    try:
        mesh = trimesh.load_mesh(filename)
        mesh.apply_scale(0.001)
        mesh.visual = ColorVisuals(mesh=mesh, face_colors=to_rgba(color))
        if rotation is not None:
            mesh.apply_transform(rotation)
        return server.scene.add_mesh_trimesh(name, mesh, position=pos)
    except Exception as e:
        print(f"Load error {name}: {e}")
        return server.scene.add_box(name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos)


class TileContext:
    """All per-tile GUI + scene state bundled together."""

    def __init__(
        self,
        server: viser.ViserServer,
        tile: Optional[SyncTile],
        tab,
        index: int,
        anchor_positions,
        m1_is_t16: bool,
        m1_carriage_offset_z: float,
        m2_carriage_offset_y: float,
        m2_carriage_offset_z: float,
        m2_rotation,
    ) -> None:
        self.server = server
        self.tile = tile
        self.index = index
        self.name = tile.name if tile is not None else f"(no tile {index})"
        self.anchor_positions = anchor_positions
        self.m1_is_t16 = m1_is_t16
        self.m2_carriage_offset_y = m2_carriage_offset_y
        self.m2_carriage_offset_z = m2_carriage_offset_z
        self.attitude_state = {"roll": 0.0, "pitch": 0.0, "init": False}
        self.color = TILE_COLORS[index % len(TILE_COLORS)]
        self.state = {"speed_m1": 255, "speed_m2": 255}

        # Per-tile scene frame. Actuator meshes are parented under this path,
        # so the whole assembly moves with the trilaterated position and the
        # IMU-derived attitude. Initial X offset so multiple tiles don't
        # overlap at the origin before UWB data arrives.
        prefix = f"tile_{index}"
        try:
            self.frame = server.scene.add_frame(
                prefix,
                position=(index * 0.4, 0.0, 0.0),
                show_axes=False,
            )
        except TypeError:
            self.frame = server.scene.add_frame(
                prefix,
                position=(index * 0.4, 0.0, 0.0),
            )

        load_mesh(
            server,
            f"{prefix}/m1_base",
            M1_CONFIG.base_stl,
            (50, 50, 50),
            (0.0, 0.0, 0.0),
        )
        self.m1_shaft = load_mesh(
            server,
            f"{prefix}/m1_shaft",
            M1_CONFIG.move_stl,
            (200, 200, 200),
            (0.0, 0.0, m1_carriage_offset_z),
        )
        load_mesh(
            server,
            f"{prefix}/m1b_base",
            M1_CONFIG.base_stl,
            (50, 50, 50),
            (-0.06, 0.0, 0.0),
        )
        self.m1b_shaft = load_mesh(
            server,
            f"{prefix}/m1b_shaft",
            M1_CONFIG.move_stl,
            (200, 200, 200),
            (-0.06, 0.0, m1_carriage_offset_z),
        )
        load_mesh(
            server,
            f"{prefix}/m2_base",
            M2_CONFIG.base_stl,
            (50, 50, 80),
            (0.1, 0.0, 0.0),
            rotation=m2_rotation,
        )
        self.m2_carriage = load_mesh(
            server,
            f"{prefix}/m2_carriage",
            M2_CONFIG.move_stl,
            (200, 200, 200),
            (0.1, m2_carriage_offset_y, m2_carriage_offset_z),
            rotation=m2_rotation,
        )
        load_mesh(
            server,
            f"{prefix}/m2b_base",
            M2_CONFIG.base_stl,
            (50, 50, 80),
            (0.16, 0.0, 0.0),
            rotation=m2_rotation,
        )
        self.m2b_carriage = load_mesh(
            server,
            f"{prefix}/m2b_carriage",
            M2_CONFIG.move_stl,
            (200, 200, 200),
            (0.16, m2_carriage_offset_y, m2_carriage_offset_z),
            rotation=m2_rotation,
        )

        # Small marker icosphere at the tile's local origin in its per-tile colour
        server.scene.add_icosphere(
            f"{prefix}/marker",
            radius=0.03,
            color=self.color,
            position=(0.0, 0.0, 0.0),
        )

        # Initialize actuators to known positions
        m1_mm_init = 0.0
        m1_z_init = (m1_mm_init - 13) / 1000.0 if m1_is_t16 else m1_mm_init / 1000.0
        self.m1_shaft.position = (0.0, 0.0, m1_z_init)

        m1b_mm_init = M1_CONFIG.stroke_mm - m1_mm_init
        m1b_z_init = (m1b_mm_init - 13) / 1000.0 if m1_is_t16 else m1b_mm_init / 1000.0
        self.m1b_shaft.position = (-0.06, 0.0, m1b_z_init)

        m2_mm_init = 0.0
        m2_y_init = m2_mm_init / 1000.0
        self.m2_carriage.position = (
            0.1,
            m2_carriage_offset_y + m2_y_init,
            m2_carriage_offset_z,
        )
        self.m2b_carriage.position = (
            0.16,
            m2_carriage_offset_y + m2_y_init,
            m2_carriage_offset_z,
        )

        # ---- Tab contents (GUI widgets) ----
        with tab:
            with server.gui.add_folder("System Controls"):
                server.gui.add_markdown(f"**Connected:** {self.name}")
                server.gui.add_button("Send Ping", color="blue").on_click(
                    lambda _: self._send("ping")
                )

            with server.gui.add_folder(f"M1: {M1_CONFIG.name}"):
                self.m1_md = server.gui.add_markdown("Waiting...")
                speed_m1 = server.gui.add_slider(
                    "Speed %", min=10, max=100, step=1, initial_value=100
                )

                @speed_m1.on_update
                def _(_):
                    self.state["speed_m1"] = int(speed_m1.value * 2.55)

                server.gui.add_button("Extend", color="green").on_click(
                    lambda _: self._send("m1_extend", self.state["speed_m1"])
                )
                server.gui.add_button("Brake", color="red").on_click(
                    lambda _: self._send("m1_brake")
                )
                server.gui.add_button("Retract", color="yellow").on_click(
                    lambda _: self._send("m1_retract", self.state["speed_m1"])
                )
                m1_slider = server.gui.add_slider(
                    "Target Position (mm)",
                    min=int(M1_CONFIG.min_position_mm),
                    max=int(M1_CONFIG.max_position_mm),
                    step=1,
                    initial_value=int(M1_CONFIG.min_position_mm),
                )
                m1_slider.on_update(
                    lambda event: self._send("m1_set_position_mm", float(event.target.value))
                )

            with server.gui.add_folder(f"M2: {M2_CONFIG.name}"):
                self.m2_md = server.gui.add_markdown("Waiting...")
                speed_m2 = server.gui.add_slider(
                    "Speed %", min=10, max=100, step=1, initial_value=100
                )

                @speed_m2.on_update
                def _(_):
                    self.state["speed_m2"] = int(speed_m2.value * 2.55)

                server.gui.add_button("Extend", color="green").on_click(
                    lambda _: self._send("m2_extend", self.state["speed_m2"])
                )
                server.gui.add_button("Brake", color="red").on_click(
                    lambda _: self._send("m2_brake")
                )
                server.gui.add_button("Retract", color="yellow").on_click(
                    lambda _: self._send("m2_retract", self.state["speed_m2"])
                )
                m2_slider = server.gui.add_slider(
                    "Target Position (mm)",
                    min=int(M2_CONFIG.min_position_mm),
                    max=int(M2_CONFIG.max_position_mm),
                    step=1,
                    initial_value=int(M2_CONFIG.min_position_mm),
                )
                m2_slider.on_update(
                    lambda event: self._send("m2_set_position_mm", float(event.target.value))
                )

            with server.gui.add_folder("Mobile Base"):
                server.gui.add_markdown("Open-loop velocity control")
                base_vx = server.gui.add_slider(
                    "Forward (vx) %", min=-100, max=100, step=1, initial_value=0
                )
                base_vy = server.gui.add_slider(
                    "Strafe (vy) %", min=-100, max=100, step=1, initial_value=0
                )
                base_omega = server.gui.add_slider(
                    "Rotate (omega) %", min=-100, max=100, step=1, initial_value=0
                )

                def send_base_velocity(_=None):
                    vx = int(base_vx.value * 1.27)
                    vy = int(base_vy.value * 1.27)
                    omega = int(base_omega.value * 1.27)
                    self._send("base_velocity", vx, vy, omega)

                server.gui.add_button("Send Velocity", color="green").on_click(send_base_velocity)
                server.gui.add_button("Brake", color="red").on_click(
                    lambda _: self._send("base_brake")
                )

                def reset_sliders(_):
                    base_vx.value = 0
                    base_vy.value = 0
                    base_omega.value = 0
                    self._send("base_brake")

                server.gui.add_button("Stop & Reset", color="yellow").on_click(reset_sliders)

            with server.gui.add_folder("UWB Localization"):
                self.uwb_md = server.gui.add_markdown("Waiting for UWB data...")

            with server.gui.add_folder("ToF Sensor"):
                self.tof_md = server.gui.add_markdown("Waiting for ToF data...")

            with server.gui.add_folder("IMU Sensor"):
                self.imu_md = server.gui.add_markdown("Waiting for IMU data...")

        if self.tile is not None:
            self.tile.on_telemetry(self._update)

    def _send(self, fn_name: str, *args) -> None:
        if self.tile is None:
            print(f"[MOCK {self.name}] {fn_name}{args}")
            return
        try:
            getattr(self.tile, fn_name)(*args)
        except Exception as e:
            print(f"[SDK {self.name}] {fn_name} failed: {e}")

    def _update(self, frame: Telemetry) -> None:
        m1_lines = [f"**Pos ADC:** {frame.m1_pos_adc} | **Est. Pos:** {frame.m1_pos_mm:.1f} mm"]
        for i, raw in enumerate(frame.m1_adcs, start=1):
            mm = (raw / 4095.0) * M1_CONFIG.stroke_mm
            m1_lines.append(f"**adc{i}:** {raw} ({mm:.1f} mm)")
        self.m1_md.content = "  \n".join(m1_lines)

        m2_lines = [f"**Pos ADC:** {frame.m2_pos_adc} | **Est. Pos:** {frame.m2_pos_mm:.1f} mm"]
        for i, raw in enumerate(frame.m2_adcs, start=1):
            mm = (raw / 4095.0) * M2_CONFIG.stroke_mm
            m2_lines.append(f"**adc{i}:** {raw} ({mm:.1f} mm)")
        self.m2_md.content = "  \n".join(m2_lines)

        # Actuator mesh positions (local to this tile's frame)
        m1_mm = frame.m1_pos_mm
        m2_mm = frame.m2_pos_mm
        m1_z = (m1_mm - 13) / 1000.0 if self.m1_is_t16 else m1_mm / 1000.0
        m1b_mm = M1_CONFIG.stroke_mm - m1_mm
        m1b_z = (m1b_mm - 13) / 1000.0 if self.m1_is_t16 else m1b_mm / 1000.0
        m2_y = m2_mm / 1000.0
        self.m1_shaft.position = (0.0, 0.0, m1_z)
        self.m1b_shaft.position = (-0.06, 0.0, m1b_z)
        self.m2_carriage.position = (
            0.1,
            self.m2_carriage_offset_y + m2_y,
            self.m2_carriage_offset_z,
        )
        self.m2b_carriage.position = (
            0.16,
            self.m2_carriage_offset_y + m2_y,
            self.m2_carriage_offset_z,
        )

        if frame.tof_mm is not None:
            self.tof_md.content = f"**Range:** {frame.tof_mm} mm"
        else:
            self.tof_md.content = "No sensor / no reading"

        if frame.uwb_mm is not None:
            d0, d1, d2 = frame.uwb_mm
            if d0 is not None and d1 is not None and d2 is not None:
                z_offset_m = ANCHOR_HEIGHT_M - TAG_HEIGHT_M
                pos = trilaterate(frame.uwb_mm, self.anchor_positions, z_offset_m=z_offset_m)
                if pos is not None:
                    self.frame.position = (pos[0], pos[1], 0.0)
                    self.uwb_md.content = (
                        f"**Pos:** ({pos[0]:.2f}, {pos[1]:.2f}) m  \n"
                        f"**Ranges:** {d0} / {d1} / {d2} mm"
                    )
                else:
                    self.uwb_md.content = f"**Ranges:** {d0} / {d1} / {d2} mm (no solution)"
            else:
                self.uwb_md.content = f"**Ranges:** {d0} / {d1} / {d2} mm (incomplete)"

        if frame.imu is not None:
            imu = frame.imu
            g = 9.80665
            self.imu_md.content = (
                f"**Accel (g):** {imu.ax/g:+.2f} {imu.ay/g:+.2f} {imu.az/g:+.2f}  \n"
                f"**Gyro (dps):** {math.degrees(imu.gx):+.1f} "
                f"{math.degrees(imu.gy):+.1f} {math.degrees(imu.gz):+.1f}"
            )
            norm = math.sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az)
            if norm > 1e-3:
                roll_meas = math.atan2(imu.ay, imu.az)
                pitch_meas = math.atan2(-imu.ax, math.sqrt(imu.ay * imu.ay + imu.az * imu.az))
                if not self.attitude_state["init"]:
                    self.attitude_state["roll"] = roll_meas
                    self.attitude_state["pitch"] = pitch_meas
                    self.attitude_state["init"] = True
                else:
                    alpha = 0.2
                    self.attitude_state["roll"] += alpha * (roll_meas - self.attitude_state["roll"])
                    self.attitude_state["pitch"] += alpha * (
                        pitch_meas - self.attitude_state["pitch"]
                    )

                roll = self.attitude_state["roll"]
                pitch = self.attitude_state["pitch"]
                cr, sr = math.cos(roll / 2), math.sin(roll / 2)
                cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
                w = cr * cp
                x = sr * cp
                y = cr * sp
                z = -sr * sp
                self.frame.wxyz = (w, x, y, z)


def main() -> None:
    parser = argparse.ArgumentParser(description="OmniTiles Debug GUI")
    parser.add_argument(
        "--tile",
        type=str,
        default=None,
        help="Specific tile name to connect to (default: first discovered).",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Connect to every OmniTile discovered during scan.",
    )
    args = parser.parse_args()

    tiles = _select_tiles(args.tile, args.all)

    anchor_positions = load_anchor_positions(ANCHOR_CONFIG_PATH)
    print(f"[GUI] Loaded {len(anchor_positions)} anchor positions from {ANCHOR_CONFIG_PATH}")

    # --- Viser Server Setup ---
    server = viser.ViserServer(label="OmniTiles Debugger")
    server.gui.configure_theme(control_width="large")
    server.scene.set_up_direction("+z")
    xs = [a[0] for a in anchor_positions] + [0.0]
    ys = [a[1] for a in anchor_positions] + [0.0]
    grid_cell = 0.1
    grid_section = 1.0
    grid_pad = 0.6
    grid_cx = round(((max(xs) + min(xs)) / 2) / grid_section) * grid_section
    grid_cy = round(((max(ys) + min(ys)) / 2) / grid_section) * grid_section
    half_w = max(max(xs) - grid_cx, grid_cx - min(xs)) + grid_pad
    half_h = max(max(ys) - grid_cy, grid_cy - min(ys)) + grid_pad
    grid_w = 2 * half_w
    grid_h = 2 * half_h

    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.initial_camera.look_at = (grid_cx, grid_cy, 0.0)
    server.initial_camera.position = (grid_cx - 3.0, grid_cy, 3.0)
    server.scene.add_grid(
        "ground",
        width=grid_w,
        height=grid_h,
        cell_size=grid_cell,
        section_size=grid_section,
        position=(grid_cx, grid_cy, 0.0),
    )

    for i, (ax, ay) in enumerate(anchor_positions):
        server.scene.add_icosphere(
            f"anchor_{i}",
            radius=0.04,
            color=(255, 120, 0),
            position=(ax, ay, 0.0),
        )
        server.scene.add_label(f"anchor_{i}_label", f"A{i}", position=(ax, ay, 0.08))

    m1_is_t16 = "t16" in M1_CONFIG.move_stl.lower()
    m1_carriage_offset_z = -0.013 if m1_is_t16 else 0.0
    m2_is_t16 = "t16" in M2_CONFIG.move_stl.lower()
    m2_carriage_offset_y = -0.013 if m2_is_t16 else 0.0
    m2_carriage_offset_z = 0.0
    m2_rotation = trimesh.transformations.rotation_matrix(math.radians(-90.0), [1.0, 0.0, 0.0])

    # One tab per tile. When no tiles were found, still create a placeholder
    # tab so the GUI is usable for scene/layout debugging.
    tab_group = server.gui.add_tab_group()
    contexts: list[TileContext] = []
    if tiles:
        for i, t in enumerate(tiles):
            tab = tab_group.add_tab(t.name)
            contexts.append(
                TileContext(
                    server,
                    t,
                    tab,
                    i,
                    anchor_positions,
                    m1_is_t16,
                    m1_carriage_offset_z,
                    m2_carriage_offset_y,
                    m2_carriage_offset_z,
                    m2_rotation,
                )
            )
    else:
        tab = tab_group.add_tab("(no tile)")
        contexts.append(
            TileContext(
                server,
                None,
                tab,
                0,
                anchor_positions,
                m1_is_t16,
                m1_carriage_offset_z,
                m2_carriage_offset_y,
                m2_carriage_offset_z,
                m2_rotation,
            )
        )

    # --- UWB Noise Characterization (shared panel with per-tile selector) ---
    ANCHOR_COLORS = ("red", "green", "blue")
    noise_state: dict = {
        "recording": False,
        "start_ms": 0,
        "duration_ms": 60_000,
        "t_ms": [],
        "d": ([], [], []),
        "plot_handle": None,
        "tile_name": None,
    }

    noise_folder = server.gui.add_folder("UWB Noise Characterization")
    with noise_folder:
        noise_md = server.gui.add_markdown(
            "Place tag at a fixed position, then Record. "
            "Collects all three anchor ranges from the selected tile for the duration."
        )
        tile_names = [c.name for c in contexts if c.tile is not None]
        dropdown_options = tile_names if tile_names else ["(no tile)"]
        noise_tile_dd = server.gui.add_dropdown(
            "Tile",
            options=dropdown_options,
            initial_value=dropdown_options[0],
        )
        noise_duration_s = server.gui.add_number("Duration (s)", initial_value=60.0, step=1.0)
        noise_record_btn = server.gui.add_button("Record")

        @noise_record_btn.on_click
        def _start_noise(_):
            if noise_state["recording"]:
                return
            noise_state["duration_ms"] = int(round(float(noise_duration_s.value) * 1000))
            noise_state["start_ms"] = int(round(time.monotonic() * 1000))
            noise_state["t_ms"] = []
            noise_state["d"] = ([], [], [])
            noise_state["tile_name"] = noise_tile_dd.value
            if noise_state["plot_handle"] is not None:
                noise_state["plot_handle"].remove()
                noise_state["plot_handle"] = None
            noise_state["recording"] = True
            noise_md.content = (
                f"Recording 0.0 / {noise_state['duration_ms']/1000:.0f} s "
                f"from {noise_state['tile_name']}..."
            )

    def noise_handler(ctx: TileContext, frame: Telemetry) -> None:
        if not noise_state["recording"]:
            return
        if noise_state["tile_name"] != ctx.name:
            return
        if frame.uwb_mm is None:
            return
        now_ms = int(round(time.monotonic() * 1000))
        elapsed_ms = now_ms - noise_state["start_ms"]
        noise_state["t_ms"].append(elapsed_ms)
        for i in range(3):
            val = frame.uwb_mm[i]
            noise_state["d"][i].append(float("nan") if val is None else float(val))
        if elapsed_ms < noise_state["duration_ms"]:
            noise_md.content = (
                f"Recording {elapsed_ms/1000:.1f} / "
                f"{noise_state['duration_ms']/1000:.0f} s "
                f"from {noise_state['tile_name']}... "
                f"({len(noise_state['t_ms'])} samples)"
            )
            return

        noise_state["recording"] = False
        t_s = np.array(noise_state["t_ms"], dtype=float) / 1000.0
        d_arrs = [np.array(noise_state["d"][i], dtype=float) for i in range(3)]

        n_samples = len(t_s)
        duration_s = float(t_s[-1]) if n_samples > 0 else 0.0
        sample_rate = n_samples / duration_s if duration_s > 0 else 0.0

        log_dir = Path(__file__).resolve().parent / "logs"
        log_dir.mkdir(exist_ok=True)
        ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = str(noise_state["tile_name"]).replace(" ", "_")
        csv_path = log_dir / f"uwb_noise_{safe_name}_{ts}.csv"
        with csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow([f"# recorded_at={_dt.datetime.now().isoformat()}"])
            w.writerow([f"# tile={noise_state['tile_name']}"])
            w.writerow([f"# anchors_m={[list(a) for a in anchor_positions]}"])
            w.writerow([f"# n_samples={n_samples}"])
            w.writerow([f"# duration_s={duration_s:.3f}"])
            w.writerow([f"# sample_rate_hz={sample_rate:.2f}"])
            w.writerow(["t_s", "d0_mm", "d1_mm", "d2_mm"])
            for k in range(n_samples):
                row = [f"{t_s[k]:.3f}"]
                for i in range(3):
                    v = d_arrs[i][k]
                    row.append("" if not np.isfinite(v) else f"{int(v)}")
                w.writerow(row)

        lines = [
            f"**UWB noise over {duration_s:.1f} s "
            f"({n_samples} samples, {sample_rate:.1f} Hz, "
            f"tile {noise_state['tile_name']})**"
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
        noise_md.content = "  \n".join(lines)

        series = (
            uplot.Series(label="t (s)"),
            uplot.Series(label="A0 (mm)", stroke=ANCHOR_COLORS[0]),
            uplot.Series(label="A1 (mm)", stroke=ANCHOR_COLORS[1]),
            uplot.Series(label="A2 (mm)", stroke=ANCHOR_COLORS[2]),
        )
        with noise_folder:
            noise_state["plot_handle"] = server.gui.add_uplot(
                data=(t_s, d_arrs[0], d_arrs[1], d_arrs[2]),
                series=series,
                title=f"UWB ranges over time ({noise_state['tile_name']})",
                aspect=2.0,
            )

    for ctx in contexts:
        if ctx.tile is not None:
            ctx.tile.on_telemetry(lambda frame, c=ctx: noise_handler(c, frame))

    print("GUI Ready at http://localhost:8080")
    try:
        while True:
            time.sleep(1)
            for ctx in contexts:
                if ctx.tile is not None and ctx.tile.connected:
                    try:
                        ctx.tile.ping()
                    except Exception as e:
                        print(f"[SDK {ctx.name}] heartbeat ping failed: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
        for ctx in contexts:
            if ctx.tile is not None:
                try:
                    ctx.tile.disconnect()
                except Exception as e:
                    print(f"[SDK {ctx.name}] disconnect failed: {e}")


if __name__ == "__main__":
    main()
