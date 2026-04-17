"""In-process fake that mimics :class:`omnitiles.SyncTile` for UI iteration.

The fake simulates actuator motion toward commanded positions and emits
synthetic telemetry so the GUI can be built and reviewed without hardware.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable

from omnitiles import M1_CONFIG, M2_CONFIG
from omnitiles.telemetry import ImuSample, Telemetry

TELEMETRY_HZ = 20.0
ACTUATOR_SPEED_MM_S = 30.0


class FakeTile:
    def __init__(
        self,
        name: str,
        xy_m: tuple[float, float],
        anchors: tuple[tuple[float, float], ...],
    ) -> None:
        self.name = name
        self.address = f"FA:KE:{abs(hash(name)) % 0xFFFFFF:06X}"
        self._xy = xy_m
        self._anchors = anchors

        self._m1_mm = M1_CONFIG.min_position_mm
        self._m2_mm = M2_CONFIG.min_position_mm
        self._m1_target: float | None = None
        self._m2_target: float | None = None
        self._m1_vel = 0.0
        self._m2_vel = 0.0

        self._connected = True
        self._callbacks: list[Callable[[Telemetry], None]] = []
        self._lock = threading.Lock()

        self._last_ts = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"fake-tile-{name}", daemon=True)
        self._thread.start()

    @property
    def connected(self) -> bool:
        return self._connected

    def on_telemetry(self, callback: Callable[[Telemetry], None]):
        with self._lock:
            self._callbacks.append(callback)

        def _unsub() -> None:
            with self._lock:
                if callback in self._callbacks:
                    self._callbacks.remove(callback)

        return _unsub

    def disconnect(self) -> None:
        self._connected = False
        self._stop.set()

    def ping(self) -> None:
        pass

    def m1_extend(self, speed: int = 255) -> None:
        with self._lock:
            self._m1_target = None
            self._m1_vel = ACTUATOR_SPEED_MM_S * (speed / 255.0)

    def m1_retract(self, speed: int = 255) -> None:
        with self._lock:
            self._m1_target = None
            self._m1_vel = -ACTUATOR_SPEED_MM_S * (speed / 255.0)

    def m1_brake(self) -> None:
        with self._lock:
            self._m1_target = None
            self._m1_vel = 0.0

    def m1_set_position(self, position: int) -> None:
        frac = position / 255.0
        mm = M1_CONFIG.min_position_mm + frac * (
            M1_CONFIG.max_position_mm - M1_CONFIG.min_position_mm
        )
        self.m1_set_position_mm(mm)

    def m1_set_position_mm(self, mm: float) -> None:
        with self._lock:
            self._m1_target = max(M1_CONFIG.min_position_mm, min(M1_CONFIG.max_position_mm, mm))

    def m2_extend(self, speed: int = 255) -> None:
        with self._lock:
            self._m2_target = None
            self._m2_vel = ACTUATOR_SPEED_MM_S * (speed / 255.0)

    def m2_retract(self, speed: int = 255) -> None:
        with self._lock:
            self._m2_target = None
            self._m2_vel = -ACTUATOR_SPEED_MM_S * (speed / 255.0)

    def m2_brake(self) -> None:
        with self._lock:
            self._m2_target = None
            self._m2_vel = 0.0

    def m2_set_position(self, position: int) -> None:
        frac = position / 255.0
        mm = M2_CONFIG.min_position_mm + frac * (
            M2_CONFIG.max_position_mm - M2_CONFIG.min_position_mm
        )
        self.m2_set_position_mm(mm)

    def m2_set_position_mm(self, mm: float) -> None:
        with self._lock:
            self._m2_target = max(M2_CONFIG.min_position_mm, min(M2_CONFIG.max_position_mm, mm))

    def base_velocity(self, vx: int, vy: int, omega: int) -> None:
        print(f"[fake {self.name}] Base velocity vx={vx} vy={vy} omega={omega}")

    def base_brake(self) -> None:
        print(f"[fake {self.name}] Base brake")

    def _step_actuator(self, pos: float, target: float | None, vel: float, dt: float, cfg) -> float:
        if target is not None:
            step = ACTUATOR_SPEED_MM_S * dt
            if pos < target:
                pos = min(target, pos + step)
            else:
                pos = max(target, pos - step)
        else:
            pos += vel * dt
        return max(cfg.min_position_mm, min(cfg.max_position_mm, pos))

    def _run(self) -> None:
        period = 1.0 / TELEMETRY_HZ
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - self._last_ts
            self._last_ts = now

            with self._lock:
                self._m1_mm = self._step_actuator(
                    self._m1_mm, self._m1_target, self._m1_vel, dt, M1_CONFIG
                )
                self._m2_mm = self._step_actuator(
                    self._m2_mm, self._m2_target, self._m2_vel, dt, M2_CONFIG
                )
                m1 = self._m1_mm
                m2 = self._m2_mm
                cbs = list(self._callbacks)

            # Synthetic telemetry.
            m1_adc = int(4095 * (m1 / M1_CONFIG.stroke_mm))
            m2_adc = int(4095 * (m2 / M2_CONFIG.stroke_mm))

            uwb_tuple: tuple[int | None, ...] | None
            if len(self._anchors) >= 3:
                ranges: list[int | None] = []
                for ax, ay in self._anchors:
                    d_m = math.hypot(ax - self._xy[0], ay - self._xy[1])
                    d_mm = int(d_m * 1000.0)
                    ranges.append(max(0, d_mm))
                uwb_tuple = tuple(ranges)
            else:
                uwb_tuple = None

            # Fake tilt angle for IMU: driven by the commanded tilt.
            tilt_frac = (m1 - M1_CONFIG.min_position_mm) / (
                M1_CONFIG.max_position_mm - M1_CONFIG.min_position_mm
            )
            tilt_rad = math.radians(-30.0 + 60.0 * tilt_frac)
            g = 9.80665
            imu = ImuSample(
                ax=-g * math.sin(tilt_rad),
                ay=0.0,
                az=g * math.cos(tilt_rad),
                gx=0.0,
                gy=0.0,
                gz=0.0,
            )

            frame = Telemetry(
                timestamp=now,
                m1_pos_adc=m1_adc,
                m2_pos_adc=m2_adc,
                m1_pos_mm=m1,
                m2_pos_mm=m2,
                m1_adcs=(m1_adc,),
                m2_adcs=(m2_adc,),
                uwb_mm=uwb_tuple,
                tof_mm=int(200.0 + 600.0 * (1.0 - m2 / M2_CONFIG.stroke_mm)),
                imu=imu,
            )
            for cb in cbs:
                try:
                    cb(frame)
                except Exception as e:
                    print(f"[fake {self.name}] Telemetry callback error: {e}")

            remaining = period - (time.monotonic() - now)
            if remaining > 0:
                self._stop.wait(remaining)


def make_fake_tiles(
    anchors: tuple[tuple[float, float], ...],
) -> list[FakeTile]:
    positions = [
        (0.4, 0.4),
        (1.0, 0.8),
        (1.4, 0.3),
    ]
    return [FakeTile(f"OmniTile_{i+1}", pos, anchors) for i, pos in enumerate(positions)]
