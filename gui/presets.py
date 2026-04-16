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


def _uniform(h: float, t: float = 0.0, n: int = 15) -> tuple[TilePreset, ...]:
    return tuple(TilePreset(h, t) for _ in range(n))


def _by_column(
    col_heights: list[float],
    col_tilts: list[float] | None = None,
    rows: int = 3,
) -> tuple[TilePreset, ...]:
    """Build presets for a row-major 3×5 grid, one value per column."""
    cols = len(col_heights)
    tilts = col_tilts or [0.0] * cols
    result: list[TilePreset] = []
    for _ in range(rows):
        for c in range(cols):
            result.append(TilePreset(col_heights[c], tilts[c]))
    return tuple(result)


PRESETS: tuple[Preset, ...] = (
    Preset("Home (Flat, Low)", _uniform(20)),
    Preset("Flat, Mid", _uniform(50)),
    Preset("Flat, High", _uniform(80)),
    Preset("Ramp Up", _by_column([30, 40, 50, 60, 70], [0, 5, 10, 15, 20])),
    Preset("Stairs", _by_column([20, 35, 50, 65, 80])),
    Preset("Tilt Forward", _uniform(50, -20)),
    Preset("Tilt Back", _uniform(50, 20)),
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
