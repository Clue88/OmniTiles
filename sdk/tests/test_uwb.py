"""Tests for the UWB trilateration helper."""

import math

from omnitiles.uwb import trilaterate

# Simple anchor grid in meters.
ANCHORS = ((0.0, 0.0), (4.0, 0.0), (0.0, 3.0))


def _dist_mm(ax: float, ay: float, px: float, py: float) -> int:
    return int(round(math.hypot(px - ax, py - ay) * 1000))


def test_trilaterate_origin():
    pt = (0.5, 0.5)
    distances = tuple(_dist_mm(a[0], a[1], pt[0], pt[1]) for a in ANCHORS)
    result = trilaterate(distances, ANCHORS)
    assert result is not None
    assert math.isclose(result[0], pt[0], abs_tol=1e-3)
    assert math.isclose(result[1], pt[1], abs_tol=1e-3)


def test_trilaterate_off_center():
    pt = (2.3, 1.7)
    distances = tuple(_dist_mm(a[0], a[1], pt[0], pt[1]) for a in ANCHORS)
    result = trilaterate(distances, ANCHORS)
    assert result is not None
    assert math.isclose(result[0], pt[0], abs_tol=2e-3)
    assert math.isclose(result[1], pt[1], abs_tol=2e-3)


def test_trilaterate_returns_none_on_missing_distance():
    assert trilaterate((1000, None, 2000), ANCHORS) is None
