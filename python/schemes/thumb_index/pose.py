from __future__ import annotations

import math
from dataclasses import dataclass, field

import pandas as pd

from ...common.dual_protocol import convert_counts_to_physical


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _wrap_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def _angle_between_vectors_deg(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    norm_a = _norm3(*a)
    norm_b = _norm3(*b)
    if norm_a <= 1e-6 or norm_b <= 1e-6:
        return 0.0

    cosine = max(
        -1.0,
        min(
            1.0,
            (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (norm_a * norm_b),
        ),
    )
    return math.degrees(math.acos(cosine))


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
class ThumbIndexStreamProcessor:
    accel_alpha: float = 0.30
    complementary_gain: float = 0.98
    thumb: SensorState = field(default_factory=SensorState)
    index: SensorState = field(default_factory=SensorState)
    last_ts_ms: int | None = None

    def update(self, frame: dict[str, float]) -> dict[str, float]:
        ts_ms = int(frame["ts_ms"])
        dt_s = self._compute_dt(ts_ms)

        thumb = self._update_single(
            state=self.thumb,
            ax=float(frame["ax1"]),
            ay=float(frame["ay1"]),
            az=float(frame["az1"]),
            gx=float(frame["gx1"]),
            gy=float(frame["gy1"]),
            gz=float(frame["gz1"]),
            dt_s=dt_s,
        )
        index = self._update_single(
            state=self.index,
            ax=float(frame["ax2"]),
            ay=float(frame["ay2"]),
            az=float(frame["az2"]),
            gx=float(frame["gx2"]),
            gy=float(frame["gy2"]),
            gz=float(frame["gz2"]),
            dt_s=dt_s,
        )

        rel_ax = thumb["ax_g"] - index["ax_g"]
        rel_ay = thumb["ay_g"] - index["ay_g"]
        rel_az = thumb["az_g"] - index["az_g"]
        rel_gravity_ax = thumb["gravity_ax_g"] - index["gravity_ax_g"]
        rel_gravity_ay = thumb["gravity_ay_g"] - index["gravity_ay_g"]
        rel_gravity_az = thumb["gravity_az_g"] - index["gravity_az_g"]
        rel_gx = thumb["gx_dps"] - index["gx_dps"]
        rel_gy = thumb["gy_dps"] - index["gy_dps"]
        rel_gz = thumb["gz_dps"] - index["gz_dps"]

        relative_motion_energy = _norm3(rel_gx, rel_gy, rel_gz)
        pair_gyro_mean = 0.5 * (thumb["gyro_norm_dps"] + index["gyro_norm_dps"])
        motion_energy = max(relative_motion_energy, pair_gyro_mean)

        rel_roll = _wrap_angle_deg(thumb["roll_deg"] - index["roll_deg"])
        rel_pitch = _wrap_angle_deg(thumb["pitch_deg"] - index["pitch_deg"])
        rel_yaw = _wrap_angle_deg(thumb["yaw_deg"] - index["yaw_deg"])
        gravity_angle = _angle_between_vectors_deg(
            (thumb["gravity_ax_g"], thumb["gravity_ay_g"], thumb["gravity_az_g"]),
            (index["gravity_ax_g"], index["gravity_ay_g"], index["gravity_az_g"]),
        )
        pose_gap = math.sqrt(rel_roll * rel_roll + rel_pitch * rel_pitch + 0.25 * rel_yaw * rel_yaw)

        return {
            "ts_ms": ts_ms,
            **{f"thumb_{key}": value for key, value in thumb.items()},
            **{f"index_{key}": value for key, value in index.items()},
            "rel_ax_g": rel_ax,
            "rel_ay_g": rel_ay,
            "rel_az_g": rel_az,
            "rel_gravity_ax_g": rel_gravity_ax,
            "rel_gravity_ay_g": rel_gravity_ay,
            "rel_gravity_az_g": rel_gravity_az,
            "rel_gx_dps": rel_gx,
            "rel_gy_dps": rel_gy,
            "rel_gz_dps": rel_gz,
            "rel_roll_deg": rel_roll,
            "rel_pitch_deg": rel_pitch,
            "rel_yaw_deg": rel_yaw,
            "gravity_angle_deg": gravity_angle,
            "pair_gyro_mean_dps": pair_gyro_mean,
            "pair_accel_mean_g": 0.5 * (thumb["accel_norm_g"] + index["accel_norm_g"]),
            "pose_gap_deg": pose_gap,
            "relative_motion_energy": relative_motion_energy,
            "motion_energy": motion_energy,
        }

    def _compute_dt(self, ts_ms: int) -> float:
        if self.last_ts_ms is None:
            self.last_ts_ms = ts_ms
            return 0.01
        dt_s = max(0.001, min(0.05, (ts_ms - self.last_ts_ms) / 1000.0))
        self.last_ts_ms = ts_ms
        return dt_s

    def _update_single(
        self,
        state: SensorState,
        ax: float,
        ay: float,
        az: float,
        gx: float,
        gy: float,
        gz: float,
        dt_s: float,
    ) -> dict[str, float]:
        if not state.initialized:
            state.ax = ax
            state.ay = ay
            state.az = az
            acc_roll = math.degrees(math.atan2(ay, az))
            acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            state.roll_deg = acc_roll
            state.pitch_deg = acc_pitch
            state.yaw_deg = 0.0
            state.initialized = True
        else:
            state.ax = self.accel_alpha * ax + (1.0 - self.accel_alpha) * state.ax
            state.ay = self.accel_alpha * ay + (1.0 - self.accel_alpha) * state.ay
            state.az = self.accel_alpha * az + (1.0 - self.accel_alpha) * state.az

        acc_roll = math.degrees(math.atan2(state.ay, state.az))
        acc_pitch = math.degrees(math.atan2(-state.ax, math.sqrt(state.ay * state.ay + state.az * state.az)))

        state.roll_deg = self.complementary_gain * (state.roll_deg + gx * dt_s) + (1.0 - self.complementary_gain) * acc_roll
        state.pitch_deg = self.complementary_gain * (state.pitch_deg + gy * dt_s) + (1.0 - self.complementary_gain) * acc_pitch
        state.yaw_deg = state.yaw_deg + gz * dt_s

        return {
            "ax_g": ax,
            "ay_g": ay,
            "az_g": az,
            "gx_dps": gx,
            "gy_dps": gy,
            "gz_dps": gz,
            "gravity_ax_g": state.ax,
            "gravity_ay_g": state.ay,
            "gravity_az_g": state.az,
            "roll_deg": state.roll_deg,
            "pitch_deg": state.pitch_deg,
            "yaw_deg": state.yaw_deg,
            "accel_norm_g": _norm3(ax, ay, az),
            "gyro_norm_dps": _norm3(gx, gy, gz),
        }


def enrich_stream(raw_data: pd.DataFrame) -> pd.DataFrame:
    if raw_data.empty:
        return pd.DataFrame()

    physical = convert_counts_to_physical(raw_data)
    processor = ThumbIndexStreamProcessor()
    rows: list[dict[str, float]] = []

    for frame in physical.to_dict(orient="records"):
        rows.append(processor.update(frame))

    return pd.DataFrame(rows)
