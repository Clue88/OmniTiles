"""3D scene rendering: anchors, tile wedge prisms with tilt visualization."""

from __future__ import annotations

import math
from typing import Any, cast

import numpy as np
import trimesh
import viser

from mapping import m1_mm_to_tilt_deg, m2_mm_to_height_cm
from state import AppState, TileState

TILE_FOOTPRINT_M = 0.27

ANCHOR_COLOR = (255, 140, 0)
TILE_BODY_COLOR = (90, 120, 170)
TILE_HIGHLIGHT_COLOR = (200, 220, 255)

_HEIGHT_EPS_M = 0.005
_TILT_EPS_DEG = 0.5


def _wedge_mesh(height_m: float, tilt_deg: float) -> trimesh.Trimesh:
    """Build a 27x27 cm prism whose top face tilts around the Y axis.

    When tilt_deg is 0 the prism is a simple box of the given height. For
    non-zero tilt, the +X edge rises and the -X edge drops, matching the
    physical picture of a ramp.
    """
    w = TILE_FOOTPRINT_M
    half = w / 2.0
    delta = half * math.tan(math.radians(tilt_deg))
    h_plus = max(0.01, height_m + delta)
    h_minus = max(0.01, height_m - delta)

    verts = np.array(
        [
            (-half, -half, 0.0),
            (half, -half, 0.0),
            (half, half, 0.0),
            (-half, half, 0.0),
            (-half, -half, h_minus),
            (half, -half, h_plus),
            (half, half, h_plus),
            (-half, half, h_minus),
        ],
        dtype=np.float64,
    )
    faces = np.array(
        [
            (0, 2, 1),
            (0, 3, 2),  # bottom (normal -z)
            (4, 5, 6),
            (4, 6, 7),  # top    (normal +z-ish)
            (0, 1, 5),
            (0, 5, 4),  # back   (-y)
            (1, 2, 6),
            (1, 6, 5),  # right  (+x)
            (2, 3, 7),
            (2, 7, 6),  # front  (+y)
            (3, 0, 4),
            (3, 4, 7),  # left   (-x)
        ],
        dtype=np.int64,
    )
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    cast(Any, mesh).visual.face_colors = (*TILE_BODY_COLOR, 255)
    return mesh


class _TileNode:
    def __init__(self, server: viser.ViserServer, index: int, name: str) -> None:
        self.server = server
        self.index = index
        self.name = name
        self.prefix = f"/tiles/tile_{index}"

        self.frame = server.scene.add_frame(
            self.prefix,
            position=(index * 0.5, 0.0, 0.0),
            show_axes=False,
        )
        self._body_handle = None
        self._label_handle = None
        self._marker_handle = None
        self._last_height_m: float | None = None
        self._last_tilt_deg: float | None = None
        self._label_visible = True

        self._rebuild(height_m=0.2, tilt_deg=0.0)
        self._marker_handle = server.scene.add_icosphere(
            f"{self.prefix}/marker",
            radius=0.025,
            color=TILE_HIGHLIGHT_COLOR,
            position=(0.0, 0.0, 0.22),
        )

    def _rebuild(self, height_m: float, tilt_deg: float) -> None:
        if self._body_handle is not None:
            self._body_handle.remove()
        mesh = _wedge_mesh(height_m, tilt_deg)
        self._body_handle = self.server.scene.add_mesh_trimesh(f"{self.prefix}/body", mesh)
        self._body_handle.on_click(lambda _e: self._toggle_label())
        self._last_height_m = height_m
        self._last_tilt_deg = tilt_deg

    def _toggle_label(self) -> None:
        self._label_visible = not self._label_visible
        if self._label_handle is not None:
            self._label_handle.visible = self._label_visible

    def update(self, st: TileState) -> None:
        if st.xy_m is not None:
            self.frame.position = (st.xy_m[0], st.xy_m[1], 0.0)

        # Use the actual telemetry-derived values so the 3D view matches
        # what the actuators are doing, not what the user commanded.
        height_cm = m2_mm_to_height_cm(st.m2_mm) if st.m2_mm is not None else st.cmd_height_cm
        tilt_deg = m1_mm_to_tilt_deg(st.m1_mm) if st.m1_mm is not None else st.cmd_tilt_deg
        height_m = height_cm / 100.0

        if (
            self._last_height_m is None
            or self._last_tilt_deg is None
            or abs(height_m - self._last_height_m) > _HEIGHT_EPS_M
            or abs(tilt_deg - self._last_tilt_deg) > _TILT_EPS_DEG
        ):
            self._rebuild(height_m, tilt_deg)

        if self._marker_handle is not None:
            self._marker_handle.position = (0.0, 0.0, height_m + 0.03)

        label_text = self._label_text(st, height_cm, tilt_deg)
        if self._label_handle is None:
            self._label_handle = self.server.scene.add_label(
                f"{self.prefix}/label",
                text=label_text,
                position=(0.0, 0.0, height_m + 0.12),
                visible=self._label_visible,
            )
        else:
            self._label_handle.position = (0.0, 0.0, height_m + 0.12)
            self._label_handle.text = label_text

    @staticmethod
    def _label_text(st: TileState, height_cm: float, tilt_deg: float) -> str:
        dot = "●" if st.connected else "○"
        return f"{dot} {st.name}\nh={height_cm:.0f}cm  θ={tilt_deg:+.0f}°"


class Scene:
    def __init__(self, server: viser.ViserServer, app_state: AppState) -> None:
        self.server = server
        self.app_state = app_state
        self._tile_nodes: dict[str, _TileNode] = {}
        self._anchor_labels: list = []

        server.scene.set_up_direction("+z")
        self._build_static()

    def _build_static(self) -> None:
        anchors = self.app_state.anchors
        xs = [a[0] for a in anchors] + [0.0]
        ys = [a[1] for a in anchors] + [0.0]
        pad = 0.4
        section = 1.0

        # Snap the grid so that (0, 0) — which is A0 by convention — lands on
        # a thicker section-line intersection. The grid is centered on
        # (cx, cy), and viser draws section lines at multiples of
        # `section` away from that center, so (0, 0) lies on an intersection
        # iff both cx and cy are integer multiples of `section`.
        raw_cx = (max(xs) + min(xs)) / 2
        raw_cy = (max(ys) + min(ys)) / 2
        cx = round(raw_cx / section) * section
        cy = round(raw_cy / section) * section
        half_w = max(max(xs) - cx, cx - min(xs)) + pad
        half_h = max(max(ys) - cy, cy - min(ys)) + pad

        # The sidebar sits on the right side of the viewport, so shift the
        # look-at point to the camera's right (-y here) so the scene drifts
        # left on screen and ends up centered in the visible 3D canvas.
        side_offset = -0.6
        self.server.initial_camera.up = (0.0, 0.0, 1.0)
        self.server.initial_camera.look_at = (cx, cy + side_offset, 0.0)
        self.server.initial_camera.position = (cx - 3.0, cy + side_offset, 3.0)

        self.server.scene.add_grid(
            "/ground",
            width=2 * half_w,
            height=2 * half_h,
            cell_size=0.1,
            section_size=section,
            position=(cx, cy, 0.0),
        )

        for i, (ax, ay) in enumerate(anchors):
            sphere = self.server.scene.add_icosphere(
                f"/anchors/a{i}",
                radius=0.05,
                color=ANCHOR_COLOR,
                position=(ax, ay, 0.0),
            )
            label = self.server.scene.add_label(
                f"/anchors/a{i}/label",
                text=f"A{i} ({ax:.2f}, {ay:.2f}) m",
                position=(0.0, 0.0, 0.12),
                visible=False,
            )
            self._anchor_labels.append(label)

            def _toggle(_event, _label=label) -> None:
                _label.visible = not _label.visible

            sphere.on_click(_toggle)

    def _ensure_node(self, st: TileState) -> _TileNode:
        node = self._tile_nodes.get(st.name)
        if node is None:
            node = _TileNode(self.server, len(self._tile_nodes), st.name)
            self._tile_nodes[st.name] = node
        return node

    def update(self) -> None:
        for st in self.app_state.tiles.values():
            node = self._ensure_node(st)
            node.update(st)
