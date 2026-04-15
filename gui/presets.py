from __future__ import annotations

from dataclasses import dataclass

from mapping import height_cm_to_m2_mm, tilt_deg_to_m1_mm


@dataclass(frozen=True)
class TilePreset:
    height_cm: float
    tilt_deg: float


@dataclass(frozen=True)
class Preset:
    name: str
    tiles: tuple[TilePreset, ...]


PRESETS: tuple[Preset, ...] = (
    Preset(
        "Home (Flat, Low)",
        (TilePreset(20, 0), TilePreset(20, 0), TilePreset(20, 0)),
    ),
    Preset(
        "Flat, Mid",
        (TilePreset(50, 0), TilePreset(50, 0), TilePreset(50, 0)),
    ),
    Preset(
        "Flat, High",
        (TilePreset(80, 0), TilePreset(80, 0), TilePreset(80, 0)),
    ),
    Preset(
        "Ramp Up",
        (TilePreset(30, 0), TilePreset(50, 10), TilePreset(70, 20)),
    ),
    Preset(
        "Stairs",
        (TilePreset(30, 0), TilePreset(50, 0), TilePreset(70, 0)),
    ),
    Preset(
        "Tilt Forward",
        (TilePreset(50, -20), TilePreset(50, -20), TilePreset(50, -20)),
    ),
    Preset(
        "Tilt Back",
        (TilePreset(50, 20), TilePreset(50, 20), TilePreset(50, 20)),
    ),
)


def preset_by_name(name: str) -> Preset | None:
    for p in PRESETS:
        if p.name == name:
            return p
    return None


def apply_preset(preset: Preset, tiles: list) -> None:
    """Apply ``preset`` to ``tiles`` in list order.

    ``tiles`` can be any list of objects that expose ``m1_set_position_mm`` and
    ``m2_set_position_mm`` (real :class:`SyncTile` or :class:`FakeTile`).
    """
    for tile, target in zip(tiles, preset.tiles):
        try:
            tile.m2_set_position_mm(height_cm_to_m2_mm(target.height_cm))
            tile.m1_set_position_mm(tilt_deg_to_m1_mm(target.tilt_deg))
        except Exception as e:
            name = getattr(tile, "name", "?")
            print(f"[preset] {name}: Apply failed: {e}")
