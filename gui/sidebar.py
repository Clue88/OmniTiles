"""Viser GUI widgets: connection, anchors, per-tile, base, presets, advanced."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

import viser

from mapping import (
    HEIGHT_MAX_CM,
    HEIGHT_MIN_CM,
    TILT_MAX_DEG,
    TILT_MIN_DEG,
    height_cm_to_m2_mm,
    tilt_deg_to_m1_mm,
)
from presets import PRESETS, apply_preset, preset_by_name
from state import AppState

TileGetter = Callable[[str], Any]
TileList = Callable[[], list]


class _TileRow:
    def __init__(
        self,
        server: viser.ViserServer,
        app_state: AppState,
        name: str,
        index: int,
        get_tile: TileGetter,
        order: float,
    ) -> None:
        self.server = server
        self.app_state = app_state
        self.name = name
        self.index = index
        self.get_tile = get_tile

        tile_state = app_state.tiles[name]

        self.folder = server.gui.add_folder(name, order=order)
        with self.folder:
            self.status_md = server.gui.add_markdown(self._status_text())

            self.height_slider = server.gui.add_slider(
                "Height (cm)",
                min=HEIGHT_MIN_CM,
                max=HEIGHT_MAX_CM,
                step=1,
                initial_value=tile_state.cmd_height_cm,
            )

            @self.height_slider.on_update
            def _(_event) -> None:
                self.app_state.tiles[self.name].cmd_height_cm = float(self.height_slider.value)

            self.tilt_slider = server.gui.add_slider(
                "Tilt (°)",
                min=TILT_MIN_DEG,
                max=TILT_MAX_DEG,
                step=1,
                initial_value=tile_state.cmd_tilt_deg,
            )

            @self.tilt_slider.on_update
            def _(_event) -> None:
                self.app_state.tiles[self.name].cmd_tilt_deg = float(self.tilt_slider.value)

            set_bg = server.gui.add_button_group("", options=("Set Height", "Set Tilt"))

            def _on_set(_e, bg=set_bg) -> None:
                if bg.value == "Set Height":
                    self._send_lift()
                elif bg.value == "Set Tilt":
                    self._send_tilt()

            set_bg.on_click(_on_set)

            if index == 0:
                self._add_mobile_base()

            with server.gui.add_folder("Debug", expand_by_default=False):
                self.debug_speed = server.gui.add_slider(
                    "Speed", min=0, max=255, step=1, initial_value=255
                )

                m1_bg = server.gui.add_button_group(
                    "Tilt (M1)",
                    options=(
                        "Extend",
                        "Brake",
                        "Retract",
                    ),
                )
                m1_bg.on_click(lambda _e, bg=m1_bg: self._on_motor_button("m1", bg))

                m2_bg = server.gui.add_button_group(
                    "Lift (M2)",
                    options=(
                        "Extend",
                        "Brake",
                        "Retract",
                    ),
                )
                m2_bg.on_click(lambda _e, bg=m2_bg: self._on_motor_button("m2", bg))

                self.adc_md = server.gui.add_markdown(self._adc_text())

                server.gui.add_button("Ping").on_click(lambda _: self._call("ping"))

    def _send_lift(self) -> None:
        tile = self.get_tile(self.name)
        if tile is None:
            print(f"[sidebar {self.name}] Not connected, skipping Go (Lift)")
            return
        h = float(self.height_slider.value)
        try:
            tile.m2_set_position_mm(height_cm_to_m2_mm(h))
        except Exception as e:
            print(f"[sidebar {self.name}] M2 command failed: {e}")

    def _send_tilt(self) -> None:
        tile = self.get_tile(self.name)
        if tile is None:
            print(f"[sidebar {self.name}] Not connected, skipping Go (Tilt)")
            return
        t = float(self.tilt_slider.value)
        try:
            tile.m1_set_position_mm(tilt_deg_to_m1_mm(t))
        except Exception as e:
            print(f"[sidebar {self.name}] M1 command failed: {e}")

    def _on_motor_button(self, prefix: str, bg) -> None:
        action = bg.value
        if action.endswith("Extend"):
            verb = "extend"
        elif action.endswith("Brake"):
            verb = "brake"
        elif action.endswith("Retract"):
            verb = "retract"
        else:
            return
        if verb == "brake":
            self._call(f"{prefix}_{verb}")
        else:
            self._call(f"{prefix}_{verb}", int(self.debug_speed.value))

    def _add_mobile_base(self) -> None:
        server = self.server
        with server.gui.add_folder("Mobile Base"):
            self.base_speed_pct = server.gui.add_slider(
                "Speed %", min=0, max=100, step=1, initial_value=30
            )

            def send(vx: int, vy: int, omega: int) -> None:
                tile = self.get_tile(self.name)
                if tile is None:
                    print("[base] No tile connected")
                    return
                s = self.base_speed_pct.value / 100.0
                try:
                    tile.base_velocity(int(vx * s), int(vy * s), int(omega * s))
                except Exception as e:
                    print(f"[base] Velocity failed: {e}")

            def brake() -> None:
                tile = self.get_tile(self.name)
                if tile is None:
                    return
                try:
                    tile.base_brake()
                except Exception as e:
                    print(f"[base] Brake failed: {e}")

            # D-pad: [↑] on top, [← ■ →] in the middle, [↓] on bottom. The
            # top/bottom rows use padding options so the arrow is centered.
            row_fwd = server.gui.add_button_group("", options=("\u2003", "↑", "\u2003\u200b"))
            row_mid = server.gui.add_button_group("Motion", options=("←", "■", "→"))
            row_back = server.gui.add_button_group(
                "", options=("\u2003\u200b\u200b", "↓", "\u2003\u200b")
            )

            def handle_fwd(_e) -> None:
                if row_fwd.value == "↑":
                    send(100, 0, 0)

            def handle_mid(_e) -> None:
                v = row_mid.value
                if v == "←":
                    send(0, -100, 0)
                elif v == "→":
                    send(0, 100, 0)
                elif v == "■":
                    brake()

            def handle_back(_e) -> None:
                if row_back.value == "↓":
                    send(-100, 0, 0)

            row_fwd.on_click(handle_fwd)
            row_mid.on_click(handle_mid)
            row_back.on_click(handle_back)

    def _call(self, method_name: str, *args) -> None:
        tile = self.get_tile(self.name)
        if tile is None:
            print(f"[sidebar {self.name}] Not connected, skipping {method_name}")
            return
        try:
            getattr(tile, method_name)(*args)
        except Exception as e:
            print(f"[sidebar {self.name}] Command {method_name} failed: {e}")

    def _status_text(self) -> str:
        st = self.app_state.tiles[self.name]
        if st.connected:
            dot = "🟢"
        elif st.last_telemetry_ts is not None:
            dot = "🔴"
        else:
            dot = "⚪"
        if st.last_telemetry_ts is not None:
            age_str = f"last update {time.monotonic() - st.last_telemetry_ts:.1f}s ago"
        else:
            age_str = "no telemetry yet"

        lines = [f"{dot} **{self.name}** · {age_str}"]

        if st.m2_mm is not None:
            from mapping import m2_mm_to_height_cm

            cur_h = m2_mm_to_height_cm(st.m2_mm)
            lines.append(f"**Height:** {cur_h:.1f} cm  (m2={st.m2_mm:.1f} mm)")
        if st.m1_mm is not None:
            from mapping import m1_mm_to_tilt_deg

            cur_t = m1_mm_to_tilt_deg(st.m1_mm)
            lines.append(f"**Tilt:** {cur_t:+.1f}°  (m1={st.m1_mm:.1f} mm)")
        if st.xy_m is not None:
            lines.append(f"**XY:** ({st.xy_m[0]:.2f}, {st.xy_m[1]:.2f}) m")
        if st.tof_height_cm is not None:
            lines.append(f"**ToF height:** {st.tof_height_cm:.1f} cm  (raw {st.tof_mm} mm)")
        if st.imu_roll_deg is not None and st.imu_pitch_deg is not None:
            lines.append(
                f"**IMU:** roll {st.imu_roll_deg:+.1f}° " f"pitch {st.imu_pitch_deg:+.1f}°"
            )

        return "  \n".join(lines)

    def _adc_text(self) -> str:
        from omnitiles import M1_CONFIG, M2_CONFIG

        st = self.app_state.tiles[self.name]
        if st.m1_pos_adc is None and st.m2_pos_adc is None:
            return "**ADC readings**  \n_no telemetry yet_"

        def _section(label: str, pos_adc: int, adcs: tuple[int, ...], stroke_mm: float) -> str:
            rows = [f"**{label}** pos_adc={pos_adc}"]
            for i, raw in enumerate(adcs, start=1):
                mm = (raw / 4095.0) * stroke_mm
                rows.append(f"- adc{i}: {raw}  ({mm:.1f} mm)")
            return "  \n".join(rows)

        sections = ["**ADC readings**"]
        if st.m1_pos_adc is not None:
            sections.append(_section("M1 (tilt)", st.m1_pos_adc, st.m1_adcs, M1_CONFIG.stroke_mm))
        if st.m2_pos_adc is not None:
            sections.append(_section("M2 (lift)", st.m2_pos_adc, st.m2_adcs, M2_CONFIG.stroke_mm))
        # Blank line between sections so each bullet list terminates cleanly
        # and the next heading doesn't get parsed as a list continuation.
        return "\n\n".join(sections)

    def refresh(self) -> None:
        self.status_md.content = self._status_text()
        self.adc_md.content = self._adc_text()


class Sidebar:
    def __init__(
        self,
        server: viser.ViserServer,
        app_state: AppState,
        get_tile: TileGetter,
        list_tiles: TileList,
        rescan: Callable[[], None],
        disconnect_all: Callable[[], None],
    ) -> None:
        self.server = server
        self.app_state = app_state
        self.get_tile = get_tile
        self.list_tiles = list_tiles
        self.rescan_cb = rescan
        self.disconnect_all_cb = disconnect_all
        self._rows: dict[str, _TileRow] = {}
        self._last_refresh = 0.0

        server.gui.configure_theme(control_width="large")
        self._inject_spacer_hider()

        self._build_connection_section()
        self._build_presets_section()
        self._build_anchors_section()
        self._build_tiles_section()
        self._build_advanced_section()

    def _inject_spacer_hider(self) -> None:
        """Hide the d-pad placeholder buttons in the rendered Mantine UI.

        Viser's button groups require every option to be a distinct string
        and always render a clickable button per option. The mobile-base
        d-pad uses unicode spaces as padding options so the arrow ends up
        centered; this script walks the DOM and makes buttons whose label
        is only whitespace / zero-width chars invisible (but still occupying
        their grid slot so the arrow stays centered).

        Viser renders add_html via React's innerHTML, which does NOT run
        <script> tags. The classic ``<img onerror>`` trick fires a handler
        when the intentionally-broken image fails to load, letting us bootstrap
        a MutationObserver that rewrites the DOM as viser renders/updates it.
        """
        js = (
            "(function(){"
            "var re=/^[\\u2003\\u200b\\s]+$/;"
            "function hide(){"
            "document.querySelectorAll('button').forEach(function(btn){"
            "if(btn.children.length<=2&&re.test(btn.textContent||'')){"
            "btn.style.visibility='hidden';"
            "btn.style.pointerEvents='none';"
            "}"
            "});"
            "}"
            "hide();setTimeout(hide,300);setTimeout(hide,1000);"
            "new MutationObserver(hide).observe("
            "document.body,{childList:true,subtree:true});"
            "})();"
            "this.remove();"
        )
        self.server.gui.add_html(f'<img src="x" style="display:none" onerror="{js}">')

    def _build_connection_section(self) -> None:
        with self.server.gui.add_folder("Connection", order=10):
            self.conn_md = self.server.gui.add_markdown("…")
            conn_bg = self.server.gui.add_button_group(
                "Actions", options=("Rescan", "Disconnect All")
            )

            def _on_conn(_e) -> None:
                if conn_bg.value == "Rescan":
                    self.rescan_cb()
                elif conn_bg.value == "Disconnect All":
                    self.disconnect_all_cb()

            conn_bg.on_click(_on_conn)

    def _build_presets_section(self) -> None:
        with self.server.gui.add_folder("Presets", order=20):
            self.preset_dd = self.server.gui.add_dropdown(
                "Preset",
                options=tuple(p.name for p in PRESETS),
                initial_value=PRESETS[0].name,
            )
            self.server.gui.add_button("Apply to all connected").on_click(
                lambda _: self._apply_selected_preset()
            )

    def _apply_selected_preset(self) -> None:
        preset = preset_by_name(self.preset_dd.value)
        if preset is None:
            return
        tiles = [t for t in self.list_tiles() if getattr(t, "connected", True)]
        apply_preset(preset, tiles)
        for tile, target in zip(tiles, preset.tiles):
            name = getattr(tile, "name", None)
            if name is None or name not in self._rows:
                continue
            self.app_state.tiles[name].cmd_height_cm = target.height_cm
            self.app_state.tiles[name].cmd_tilt_deg = target.tilt_deg
            self._rows[name].height_slider.value = target.height_cm
            self._rows[name].tilt_slider.value = target.tilt_deg

    def _build_anchors_section(self) -> None:
        with self.server.gui.add_folder("Anchors", expand_by_default=False, order=30):
            lines = []
            for i, (x, y) in enumerate(self.app_state.anchors):
                lines.append(f"**A{i}:** ({x:.2f}, {y:.2f}) m")
            self.server.gui.add_markdown("  \n".join(lines))

    def _build_tiles_section(self) -> None:
        self._tiles_placeholder = self.server.gui.add_markdown("_No tiles yet — scan to discover._")
        self._sync_tile_rows()

    def _sync_tile_rows(self) -> None:
        for name in list(self.app_state.tiles.keys()):
            if name in self._rows:
                continue
            index = len(self._rows)
            self._rows[name] = _TileRow(
                self.server,
                self.app_state,
                name,
                index,
                self.get_tile,
                order=40 + index,
            )
        if self._rows and self._tiles_placeholder is not None:
            try:
                self._tiles_placeholder.remove()
            except Exception:
                pass
            self._tiles_placeholder = None

    def _build_advanced_section(self) -> None:
        self.advanced_folder = self.server.gui.add_folder(
            "Advanced", expand_by_default=False, order=1000
        )

    def refresh(self) -> None:
        now = time.monotonic()
        if now - self._last_refresh < 0.2:
            return
        self._last_refresh = now
        self._sync_tile_rows()
        self._refresh_connection()
        for row in self._rows.values():
            row.refresh()

    def _refresh_connection(self) -> None:
        lines = []
        if self.app_state.scanning:
            lines.append("_Scanning…_")
        if not self.app_state.tiles:
            lines.append("No tiles yet.")
        else:
            for name, st in self.app_state.tiles.items():
                if st.connected:
                    dot = "🟢"
                elif st.last_telemetry_ts is not None:
                    dot = "🔴"
                else:
                    dot = "⚪"
                if st.last_telemetry_ts is not None:
                    age = f"last update {time.monotonic() - st.last_telemetry_ts:.1f}s ago"
                else:
                    age = "no telemetry yet"
                lines.append(f"{dot} **{name}** · {age}")
        self.conn_md.content = "  \n".join(lines)
