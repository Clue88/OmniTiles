"""UWB multilateration helper."""

from collections.abc import Sequence

import numpy as np

from omnitiles.hardware import DEFAULT_ANCHOR_POSITIONS

Point2D = tuple[float, float]


def trilaterate(
    distances_mm: Sequence[int | None],
    anchors: Sequence[Point2D] = DEFAULT_ANCHOR_POSITIONS,
    z_offset_m: float = 0.0,
) -> Point2D | None:
    """Compute a 2D position from anchor distances using least-squares.

    Works with 3 or more anchors. When more than 3 valid distances are
    available the overdetermined system is solved via least-squares,
    averaging out per-anchor ranging noise.

    Args:
        distances_mm: Distances from each anchor, in millimeters.
            ``None`` entries are skipped; at least 3 valid distances are
            required.
        anchors: Anchor ``(x, y)`` positions in meters. Must have the
            same length as *distances_mm*.
        z_offset_m: Vertical distance between the tag and the (shared)
            anchor plane, in meters. Measured 3D ranges are projected
            onto the floor plane via ``r = sqrt(d**2 - z**2)`` before
            solving.

    Returns:
        ``(x, y)`` in meters, or ``None`` if fewer than 3 valid ranges
        or any valid range is shorter than *z_offset_m*.
    """
    if len(distances_mm) != len(anchors):
        raise ValueError(
            f"distances_mm length ({len(distances_mm)}) != "
            f"anchors length ({len(anchors)})"
        )

    # Filter to anchors with valid distances.
    valid = [
        (anchors[i], distances_mm[i])
        for i in range(len(distances_mm))
        if distances_mm[i] is not None
    ]
    if len(valid) < 3:
        return None

    a = np.array([v[0] for v in valid], dtype=float)
    d_3d = np.array([v[1] / 1000.0 for v in valid], dtype=float)

    horiz_sq = d_3d**2 - z_offset_m**2
    if np.any(horiz_sq < 0):
        return None
    d = np.sqrt(horiz_sq)

    # Linearize by subtracting the first anchor's equation from the rest.
    x0, y0 = a[0]
    lhs = np.array(
        [[2 * (a[i, 0] - x0), 2 * (a[i, 1] - y0)] for i in range(1, len(a))]
    )
    rhs = np.array(
        [
            d[0] ** 2 - d[i] ** 2 - x0**2 + a[i, 0] ** 2 - y0**2 + a[i, 1] ** 2
            for i in range(1, len(a))
        ]
    )

    sol, _, _, _ = np.linalg.lstsq(lhs, rhs, rcond=None)
    return float(sol[0]), float(sol[1])
