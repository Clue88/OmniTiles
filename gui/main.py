import argparse
import math
import os
import time

import trimesh
import viser
from trimesh.visual import to_rgba
from trimesh.visual.color import ColorVisuals

from omnitiles import (
    DEFAULT_ANCHOR_POSITIONS,
    M1_CONFIG,
    M2_CONFIG,
    SyncTile,
    Telemetry,
    TileInfo,
    scan_sync,
    trilaterate,
)


def _select_tile(name: str | None) -> SyncTile:
    infos = scan_sync()
    if not infos:
        print("[SDK] No OmniTile devices found. Starting GUI with no tile.")
        return None  # type: ignore[return-value]

    if name is not None:
        matches = [i for i in infos if i.name == name]
        if not matches:
            print(f"[SDK] Tile {name!r} not found. Available: {[i.name for i in infos]}")
            return None  # type: ignore[return-value]
        info: TileInfo = matches[0]
    else:
        info = infos[0]

    print(f"[SDK] Connecting to {info.name} ({info.address})...")
    return SyncTile.connect(info)


def main() -> None:
    parser = argparse.ArgumentParser(description="OmniTiles Debug GUI")
    parser.add_argument(
        "--tile",
        type=str,
        default=None,
        help="Specific tile name to connect to (default: first discovered).",
    )
    args = parser.parse_args()

    tile = _select_tile(args.tile)

    # --- Viser Server Setup ---
    server = viser.ViserServer(label="OmniTiles Debugger")
    server.gui.configure_theme(control_width="large")
    server.scene.add_grid("ground", width=0.5, height=0.5, cell_size=0.05)

    def load_mesh(name, filename, color, pos, rotation=None):
        if not os.path.exists(filename):
            return server.scene.add_box(
                name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
            )
        try:
            mesh = trimesh.load_mesh(filename)
            mesh.apply_scale(0.001)
            mesh.visual = ColorVisuals(mesh=mesh, face_colors=to_rgba(color))
            if rotation is not None:
                mesh.apply_transform(rotation)
            return server.scene.add_mesh_trimesh(name, mesh, position=pos)
        except Exception as e:
            print(f"Load error {name}: {e}")
            return server.scene.add_box(
                name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
            )

    # Load M1 Models
    m1_is_t16 = "t16" in M1_CONFIG.move_stl.lower()
    m1_carriage_offset_z = -0.013 if m1_is_t16 else 0.0

    # Primary M1
    load_mesh("m1_base", M1_CONFIG.base_stl, color=(50, 50, 50), pos=(0.0, 0.0, 0.0))
    m1_shaft = load_mesh(
        "m1_shaft",
        M1_CONFIG.move_stl,
        color=(200, 200, 200),
        pos=(0.0, 0.0, m1_carriage_offset_z),
    )

    # Secondary M1, mirrored in height (150 - x) and offset in X
    load_mesh("m1b_base", M1_CONFIG.base_stl, color=(50, 50, 50), pos=(-0.06, 0.0, 0.0))
    m1b_shaft = load_mesh(
        "m1b_shaft",
        M1_CONFIG.move_stl,
        color=(200, 200, 200),
        pos=(-0.06, 0.0, m1_carriage_offset_z),
    )

    # Load M2 Models (horizontal orientation along Y)
    m2_is_t16 = "t16" in M2_CONFIG.move_stl.lower()
    m2_carriage_offset_y = -0.013 if m2_is_t16 else 0.0
    m2_carriage_offset_z = 0.0
    m2_rotation = trimesh.transformations.rotation_matrix(math.radians(-90.0), [1.0, 0.0, 0.0])

    load_mesh(
        "m2_base",
        M2_CONFIG.base_stl,
        color=(50, 50, 80),
        pos=(0.1, 0.0, 0.0),
        rotation=m2_rotation,
    )
    m2_carriage = load_mesh(
        "m2_carriage",
        M2_CONFIG.move_stl,
        color=(200, 200, 200),
        pos=(0.1, m2_carriage_offset_y, m2_carriage_offset_z),
        rotation=m2_rotation,
    )

    load_mesh(
        "m2b_base",
        M2_CONFIG.base_stl,
        color=(50, 50, 80),
        pos=(0.16, 0.0, 0.0),
        rotation=m2_rotation,
    )
    m2b_carriage = load_mesh(
        "m2b_carriage",
        M2_CONFIG.move_stl,
        color=(200, 200, 200),
        pos=(0.16, m2_carriage_offset_y, m2_carriage_offset_z),
        rotation=m2_rotation,
    )

    # Initialize actuators to known positions
    m1_mm_init = 0.0
    m1_z_init = (m1_mm_init - 13) / 1000.0 if m1_is_t16 else m1_mm_init / 1000.0
    m1_shaft.position = (0.0, 0.0, m1_z_init)

    m1b_mm_init = M1_CONFIG.stroke_mm - m1_mm_init
    m1b_z_init = (m1b_mm_init - 13) / 1000.0 if m1_is_t16 else m1b_mm_init / 1000.0
    m1b_shaft.position = (-0.06, 0.0, m1b_z_init)

    m2_mm_init = 0.0
    m2_y_init = m2_mm_init / 1000.0
    m2_carriage.position = (0.1, m2_carriage_offset_y + m2_y_init, m2_carriage_offset_z)
    m2b_carriage.position = (0.16, m2_carriage_offset_y + m2_y_init, m2_carriage_offset_z)

    # UWB tile position marker (also shows IMU-derived orientation)
    tile_marker = server.scene.add_icosphere(
        "tile_position", radius=0.03, color=(0, 200, 255), position=(0.0, 0.0, 0.0)
    )
    tile_marker.visible = False

    attitude_state = {"roll": 0.0, "pitch": 0.0, "init": False}

    # --- Command dispatch ---
    def send(fn_name: str, *args) -> None:
        if tile is None:
            print(f"[MOCK] {fn_name}{args}")
            return
        try:
            getattr(tile, fn_name)(*args)
        except Exception as e:
            print(f"[SDK] {fn_name} failed: {e}")

    state = {"speed_m1": 255, "speed_m2": 255}

    with server.gui.add_folder("System Controls"):
        title = tile.name if tile is not None else "(no tile)"
        server.gui.add_markdown(f"**Connected:** {title}")
        server.gui.add_button("Send Ping", color="blue").on_click(lambda _: send("ping"))

    with server.gui.add_folder(f"M1: {M1_CONFIG.name}"):
        m1_md = server.gui.add_markdown("Waiting...")
        speed_m1 = server.gui.add_slider("Speed %", min=10, max=100, step=1, initial_value=100)

        @speed_m1.on_update
        def _(_):
            state["speed_m1"] = int(speed_m1.value * 2.55)

        server.gui.add_button("Extend", color="green").on_click(
            lambda _: send("m1_extend", state["speed_m1"])
        )
        server.gui.add_button("Brake", color="red").on_click(lambda _: send("m1_brake"))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send("m1_retract", state["speed_m1"])
        )
        m1_slider = server.gui.add_slider(
            "Target Position (mm)",
            min=int(M1_CONFIG.min_position_mm),
            max=int(M1_CONFIG.max_position_mm),
            step=1,
            initial_value=int(M1_CONFIG.min_position_mm),
        )
        m1_slider.on_update(lambda event: send("m1_set_position_mm", float(event.target.value)))

    with server.gui.add_folder(f"M2: {M2_CONFIG.name}"):
        m2_md = server.gui.add_markdown("Waiting...")
        speed_m2 = server.gui.add_slider("Speed %", min=10, max=100, step=1, initial_value=100)

        @speed_m2.on_update
        def _(_):
            state["speed_m2"] = int(speed_m2.value * 2.55)

        server.gui.add_button("Extend", color="green").on_click(
            lambda _: send("m2_extend", state["speed_m2"])
        )
        server.gui.add_button("Brake", color="red").on_click(lambda _: send("m2_brake"))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send("m2_retract", state["speed_m2"])
        )
        m2_slider = server.gui.add_slider(
            "Target Position (mm)",
            min=int(M2_CONFIG.min_position_mm),
            max=int(M2_CONFIG.max_position_mm),
            step=1,
            initial_value=int(M2_CONFIG.min_position_mm),
        )
        m2_slider.on_update(lambda event: send("m2_set_position_mm", float(event.target.value)))

    with server.gui.add_folder("Mobile Base"):
        server.gui.add_markdown("Open-loop velocity control")
        base_vx = server.gui.add_slider(
            "Forward (vx) %", min=-100, max=100, step=1, initial_value=0
        )
        base_vy = server.gui.add_slider("Strafe (vy) %", min=-100, max=100, step=1, initial_value=0)
        base_omega = server.gui.add_slider(
            "Rotate (omega) %", min=-100, max=100, step=1, initial_value=0
        )

        def send_base_velocity(_=None):
            vx = int(base_vx.value * 1.27)
            vy = int(base_vy.value * 1.27)
            omega = int(base_omega.value * 1.27)
            send("base_velocity", vx, vy, omega)

        server.gui.add_button("Send Velocity", color="green").on_click(send_base_velocity)
        server.gui.add_button("Brake", color="red").on_click(lambda _: send("base_brake"))

        def reset_sliders(_):
            base_vx.value = 0
            base_vy.value = 0
            base_omega.value = 0
            send("base_brake")

        server.gui.add_button("Stop & Reset", color="yellow").on_click(reset_sliders)

    with server.gui.add_folder("UWB Localization"):
        uwb_md = server.gui.add_markdown("Waiting for UWB data...")

    with server.gui.add_folder("ToF Sensor"):
        tof_md = server.gui.add_markdown("Waiting for ToF data...")

    with server.gui.add_folder("IMU (LSM6DSV16X)"):
        imu_md = server.gui.add_markdown("Waiting for IMU data...")

    # --- Telemetry handler ---
    def update_ui(frame: Telemetry) -> None:
        # Motor positions
        m1_lines = [
            f"**Pos ADC:** {frame.m1_pos_adc} | **Est. Pos:** {frame.m1_pos_mm:.1f} mm"
        ]
        for i, raw in enumerate(frame.m1_adcs, start=1):
            mm = (raw / 4095.0) * M1_CONFIG.stroke_mm
            m1_lines.append(f"**adc{i}:** {raw} ({mm:.1f} mm)")
        m1_md.content = "  \n".join(m1_lines)

        m2_lines = [
            f"**Pos ADC:** {frame.m2_pos_adc} | **Est. Pos:** {frame.m2_pos_mm:.1f} mm"
        ]
        for i, raw in enumerate(frame.m2_adcs, start=1):
            mm = (raw / 4095.0) * M2_CONFIG.stroke_mm
            m2_lines.append(f"**adc{i}:** {raw} ({mm:.1f} mm)")
        m2_md.content = "  \n".join(m2_lines)

        m1_mm = frame.m1_pos_mm
        m2_mm = frame.m2_pos_mm
        m1_z = (m1_mm - 13) / 1000.0 if m1_is_t16 else m1_mm / 1000.0
        m1b_mm = M1_CONFIG.stroke_mm - m1_mm
        m1b_z = (m1b_mm - 13) / 1000.0 if m1_is_t16 else m1b_mm / 1000.0
        m2_y = m2_mm / 1000.0
        m1_shaft.position = (0.0, 0.0, m1_z)
        m1b_shaft.position = (-0.06, 0.0, m1b_z)
        m2_carriage.position = (0.1, m2_carriage_offset_y + m2_y, m2_carriage_offset_z)
        m2b_carriage.position = (0.16, m2_carriage_offset_y + m2_y, m2_carriage_offset_z)

        # ToF
        if frame.tof_mm is not None:
            tof_md.content = f"**Range:** {frame.tof_mm} mm"
        else:
            tof_md.content = "No sensor / no reading"

        # UWB
        if frame.uwb_mm is not None:
            d0, d1, d2 = frame.uwb_mm
            if d0 is not None and d1 is not None and d2 is not None:
                pos = trilaterate(frame.uwb_mm, DEFAULT_ANCHOR_POSITIONS)
                if pos is not None:
                    tile_marker.position = (pos[0], pos[1], 0.0)
                    tile_marker.visible = True
                    uwb_md.content = (
                        f"**Pos:** ({pos[0]:.2f}, {pos[1]:.2f}) m  \n"
                        f"**Ranges:** {d0} / {d1} / {d2} mm"
                    )
            else:
                uwb_md.content = (
                    f"**Ranges:** {d0} / {d1} / {d2} mm (incomplete)"
                )

        # IMU + attitude
        if frame.imu is not None:
            imu = frame.imu
            g = 9.80665
            imu_md.content = (
                f"**Accel (g):** {imu.ax/g:+.2f} {imu.ay/g:+.2f} {imu.az/g:+.2f}  \n"
                f"**Gyro (dps):** {math.degrees(imu.gx):+.1f} "
                f"{math.degrees(imu.gy):+.1f} {math.degrees(imu.gz):+.1f}"
            )
            norm = math.sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az)
            if norm > 1e-3:
                roll_meas = math.atan2(imu.ay, imu.az)
                pitch_meas = math.atan2(-imu.ax, math.sqrt(imu.ay * imu.ay + imu.az * imu.az))
                if not attitude_state["init"]:
                    attitude_state["roll"] = roll_meas
                    attitude_state["pitch"] = pitch_meas
                    attitude_state["init"] = True
                else:
                    alpha = 0.2
                    attitude_state["roll"] += alpha * (roll_meas - attitude_state["roll"])
                    attitude_state["pitch"] += alpha * (pitch_meas - attitude_state["pitch"])

                roll = attitude_state["roll"]
                pitch = attitude_state["pitch"]
                cr, sr = math.cos(roll / 2), math.sin(roll / 2)
                cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
                w = cr * cp
                x = sr * cp
                y = cr * sp
                z = -sr * sp
                tile_marker.wxyz = (w, x, y, z)
                tile_marker.visible = True

    if tile is not None:
        tile.on_telemetry(update_ui)

    print("GUI Ready at http://localhost:8080")
    try:
        while True:
            time.sleep(1)
            if tile is not None and tile.connected:
                try:
                    tile.ping()
                except Exception as e:
                    print(f"[SDK] heartbeat ping failed: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
        if tile is not None:
            tile.disconnect()


if __name__ == "__main__":
    main()
