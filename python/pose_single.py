from __future__ import annotations

import math
from dataclasses import dataclass, field

import pandas as pd

from protocol_single import convert_counts_to_physical


@dataclass
class SensorState:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    initialized: bool = False


@dataclass
class SingleIMUStreamProcessor:
    accel_alpha: float = 0.30
    complementary_gain: float = 0.98
    sensor: SensorState = field(default_factory=SensorState)
    last_ts_ms: int | None = None

    def update(self, frame: dict[str, float]) -> dict[str, float]:
        ts_ms = int(frame["ts_ms"])
        dt_s = self._compute_dt(ts_ms)

        ax = float(frame["ax"])
        ay = float(frame["ay"])
        az = float(frame["az"])
        gx = float(frame["gx"])
        gy = float(frame["gy"])
        gz = float(frame["gz"])

        if not self.sensor.initialized:
            self.sensor.ax = ax
            self.sensor.ay = ay
            self.sensor.az = az
            acc_roll = math.degrees(math.atan2(ay, az))
            acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            self.sensor.roll_deg = acc_roll
            self.sensor.pitch_deg = acc_pitch
            self.sensor.yaw_deg = 0.0
            self.sensor.initialized = True
        else:
            self.sensor.ax = self.accel_alpha * ax + (1.0 - self.accel_alpha) * self.sensor.ax
            self.sensor.ay = self.accel_alpha * ay + (1.0 - self.accel_alpha) * self.sensor.ay
            self.sensor.az = self.accel_alpha * az + (1.0 - self.accel_alpha) * self.sensor.az

        acc_roll = math.degrees(math.atan2(self.sensor.ay, self.sensor.az))
        acc_pitch = math.degrees(
            math.atan2(-self.sensor.ax, math.sqrt(self.sensor.ay * self.sensor.ay + self.sensor.az * self.sensor.az))
        )

        self.sensor.roll_deg = (
            self.complementary_gain * (self.sensor.roll_deg + gx * dt_s)
            + (1.0 - self.complementary_gain) * acc_roll
        )
        self.sensor.pitch_deg = (
            self.complementary_gain * (self.sensor.pitch_deg + gy * dt_s)
            + (1.0 - self.complementary_gain) * acc_pitch
        )
        self.sensor.yaw_deg = self.sensor.yaw_deg + gz * dt_s

        accel_norm_g = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_norm_dps = math.sqrt(gx * gx + gy * gy + gz * gz)
        if accel_norm_g <= 1e-6:
            tilt_deg = 0.0
        else:
            cosine = max(-1.0, min(1.0, az / accel_norm_g))
            tilt_deg = math.degrees(math.acos(cosine))

        return {
            "ts_ms": ts_ms,
            "ax_g": ax,
            "ay_g": ay,
            "az_g": az,
            "gx_dps": gx,
            "gy_dps": gy,
            "gz_dps": gz,
            "roll_deg": self.sensor.roll_deg,
            "pitch_deg": self.sensor.pitch_deg,
            "yaw_deg": self.sensor.yaw_deg,
            "accel_norm_g": accel_norm_g,
            "gyro_norm_dps": gyro_norm_dps,
            "tilt_deg": tilt_deg,
            "motion_energy": gyro_norm_dps,
        }

    def _compute_dt(self, ts_ms: int) -> float:
        if self.last_ts_ms is None:
            self.last_ts_ms = ts_ms
            return 0.01
        dt_s = max(0.001, min(0.05, (ts_ms - self.last_ts_ms) / 1000.0))
        self.last_ts_ms = ts_ms
        return dt_s


def enrich_stream(raw_data: pd.DataFrame) -> pd.DataFrame:
    if raw_data.empty:
        return pd.DataFrame()

    physical = convert_counts_to_physical(raw_data)
    processor = SingleIMUStreamProcessor()
    rows: list[dict[str, float]] = []

    for frame in physical.to_dict(orient="records"):
        rows.append(processor.update(frame))

    return pd.DataFrame(rows)
