"""UWB multilateration and EKF position filter."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from omnitiles.hardware import DEFAULT_ANCHOR_POSITIONS
from omnitiles.telemetry import ImuSample

Point2D = tuple[float, float]


# ---------------------------------------------------------------------------
# EKF for UWB + IMU position fusion
# ---------------------------------------------------------------------------


class UwbEkf:
    """Extended Kalman Filter fusing UWB ranges with IMU motion detection.

    State vector: ``[px, py, vx, vy]`` — 2D position and velocity in meters.

    *   **Predict** uses a constant-velocity model.  Process noise *Q* is
        modulated by the IMU: when the accelerometer/gyroscope indicate the
        tile is stationary, *Q* is very small so the filter aggressively
        smooths UWB noise.  When motion is detected, *Q* grows so the filter
        tracks the new position.
    *   **Update** ingests one or more UWB range observations.  Each range is
        individually gated (Mahalanobis chi² test) to reject multipath
        outliers before the state is updated.
    """

    GRAVITY = 9.81
    # IMU thresholds for translational motion detection.  Only |gz| (yaw
    # rate) is used for the gyro check because the IMU is on the tilting
    # plate — tilt actuator motion causes gx/gy spikes that are not
    # translational.
    ACCEL_MOTION_THRESH = 1.0  # m/s² deviation from gravity norm
    GYRO_YAW_MOTION_THRESH = 0.1  # rad/s (|gz| only)
    SIGMA_A_STATIONARY = 0.01  # m/s² — very tight when IMU confirms no motion
    SIGMA_A_MOVING = 5.0  # m/s² — loose, ~1 s to track new motion
    GATE_CHI2 = 9.0  # chi² gate (1-DOF, ~99.7%)

    def __init__(
        self,
        anchors: Sequence[Point2D] = DEFAULT_ANCHOR_POSITIONS,
        range_std_m: float | Sequence[float] = 0.07,
    ) -> None:
        self._anchors = np.array(anchors, dtype=float)
        self._n_anchors = len(anchors)
        if isinstance(range_std_m, (int, float)):
            self._R_diag = np.full(self._n_anchors, float(range_std_m) ** 2)
        else:
            self._R_diag = np.array([float(s) ** 2 for s in range_std_m], dtype=float)
        self._x: np.ndarray | None = None
        self._P: np.ndarray | None = None
        self._last_t: float | None = None

    @property
    def initialized(self) -> bool:
        return self._x is not None

    @property
    def position(self) -> Point2D | None:
        if self._x is None:
            return None
        return (float(self._x[0]), float(self._x[1]))

    @property
    def velocity(self) -> tuple[float, float] | None:
        if self._x is None:
            return None
        return (float(self._x[2]), float(self._x[3]))

    def reset(self) -> None:
        self._x = None
        self._P = None
        self._last_t = None

    # ----- public entry point -----

    def step(
        self,
        distances_mm: Sequence[int | None],
        timestamp: float,
        imu: ImuSample | None = None,
        z_offset_m: float = 0.0,
    ) -> Point2D | None:
        """Run one predict + update cycle.

        Returns the filtered ``(x, y)`` in meters, or ``None`` if the filter
        has not yet been initialised (needs at least one trilateration fix).
        """
        if self._x is None:
            if not self._try_init(distances_mm, z_offset_m, timestamp):
                return None

        self._predict(timestamp, imu)
        self._update(distances_mm, z_offset_m)
        self._last_t = timestamp
        return self.position

    # ----- internals -----

    def _try_init(
        self,
        distances_mm: Sequence[int | None],
        z_offset_m: float,
        t: float,
    ) -> bool:
        anchor_tuples = [(float(a[0]), float(a[1])) for a in self._anchors]
        pos = trilaterate(distances_mm, anchor_tuples, z_offset_m)
        if pos is None:
            return False
        self._x = np.array([pos[0], pos[1], 0.0, 0.0])
        self._P = np.diag([0.1, 0.1, 0.01, 0.01])
        self._last_t = t
        return True

    def _sigma_a(self, imu: ImuSample | None) -> float:
        if imu is None:
            return self.SIGMA_A_MOVING
        accel_norm = math.sqrt(imu.ax**2 + imu.ay**2 + imu.az**2)
        accel_dev = abs(accel_norm - self.GRAVITY)
        # Squared ratio so small biases (e.g. 0.15 m/s² accel offset)
        # contribute negligibly while real motion saturates quickly.
        a_factor = (accel_dev / self.ACCEL_MOTION_THRESH) ** 2
        g_factor = (abs(imu.gz) / self.GYRO_YAW_MOTION_THRESH) ** 2
        motion = min(max(a_factor, g_factor), 1.0)
        # Dead zone: zero out the small residual from accel bias / sensor noise
        # so stationary Q is truly SIGMA_A_STATIONARY.
        if motion < 0.05:
            motion = 0.0
        return self.SIGMA_A_STATIONARY + motion * (self.SIGMA_A_MOVING - self.SIGMA_A_STATIONARY)

    def _predict(self, t: float, imu: ImuSample | None) -> None:
        dt = t - self._last_t if self._last_t is not None else 0.1
        dt = max(dt, 1e-4)

        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt

        sa = self._sigma_a(imu)
        q = sa**2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        Q = q * np.array(
            [
                [dt4 / 4, 0, dt3 / 2, 0],
                [0, dt4 / 4, 0, dt3 / 2],
                [dt3 / 2, 0, dt2, 0],
                [0, dt3 / 2, 0, dt2],
            ]
        )

        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q

    def _update(self, distances_mm: Sequence[int | None], z_offset_m: float) -> None:
        px, py = self._x[0], self._x[1]
        for i in range(min(self._n_anchors, len(distances_mm))):
            if distances_mm[i] is None:
                continue
            d_m = distances_mm[i] / 1000.0
            horiz_sq = d_m**2 - z_offset_m**2
            if horiz_sq <= 0:
                continue
            z_meas = math.sqrt(horiz_sq)

            ax, ay = self._anchors[i]
            dx = float(px) - ax
            dy = float(py) - ay
            pred_d = math.sqrt(dx * dx + dy * dy)
            if pred_d < 1e-6:
                pred_d = 1e-6

            H = np.array([[dx / pred_d, dy / pred_d, 0.0, 0.0]])
            innov = z_meas - pred_d
            S = float((H @ self._P @ H.T)[0, 0]) + self._R_diag[i]

            # Mahalanobis gate — reject outliers
            if innov * innov / S > self.GATE_CHI2:
                continue

            K = (self._P @ H.T) / S  # (4,1)
            self._x = self._x + K.ravel() * innov
            self._P = (np.eye(4) - K @ H) @ self._P
            px, py = self._x[0], self._x[1]


# ---------------------------------------------------------------------------
# Least-squares trilateration (single-shot, no state)
# ---------------------------------------------------------------------------


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
