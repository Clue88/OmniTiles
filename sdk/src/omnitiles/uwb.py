"""UWB trilateration helper."""

from collections.abc import Sequence

import numpy as np

from omnitiles.hardware import DEFAULT_ANCHOR_POSITIONS

Point2D = tuple[float, float]


def trilaterate(
    distances_mm: Sequence[int | None],
    anchors: Sequence[Point2D] = DEFAULT_ANCHOR_POSITIONS,
) -> Point2D | None:
    """Compute a 2D position from three anchor distances.

    Args:
        distances_mm: Distances from each of three anchors, in millimeters.
            Any ``None`` entry disables the solver and returns ``None``.
        anchors: Anchor ``(x, y)`` positions in meters. Defaults to
            :data:`omnitiles.hardware.DEFAULT_ANCHOR_POSITIONS`.

    Returns:
        ``(x, y)`` in meters, or ``None`` if the system is underdetermined.
    """
    if len(distances_mm) != 3 or any(d is None for d in distances_mm):
        return None
    if len(anchors) != 3:
        raise ValueError("trilaterate requires exactly 3 anchors")

    d = np.array([mm / 1000.0 for mm in distances_mm], dtype=float)
    a = np.asarray(anchors, dtype=float)
    (x0, y0), (x1, y1), (x2, y2) = a

    lhs = np.array(
        [
            [2 * (x1 - x0), 2 * (y1 - y0)],
            [2 * (x2 - x0), 2 * (y2 - y0)],
        ]
    )
    rhs = np.array(
        [
            d[0] ** 2 - d[1] ** 2 - x0**2 + x1**2 - y0**2 + y1**2,
            d[0] ** 2 - d[2] ** 2 - x0**2 + x2**2 - y0**2 + y2**2,
        ]
    )
    try:
        sol = np.linalg.solve(lhs, rhs)
    except np.linalg.LinAlgError:
        return None
    return float(sol[0]), float(sol[1])
