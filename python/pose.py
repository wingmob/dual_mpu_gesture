from __future__ import annotations

import math
from dataclasses import dataclass, field

import pandas as pd

from protocol import convert_counts_to_physical


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
class DualIMUStreamProcessor:
    accel_alpha: float = 0.30
    complementary_gain: float = 0.98
    hand: SensorState = field(default_factory=SensorState)
    forearm: SensorState = field(default_factory=SensorState)
    last_ts_ms: int | None = None

    def update(self, frame: dict[str, float]) -> dict[str, float]:
        ts_ms = int(frame["ts_ms"])
        dt_s = self._compute_dt(ts_ms)

        hand = self._update_single(
            state=self.hand,
            ax=float(frame["ax1"]),
            ay=float(frame["ay1"]),
            az=float(frame["az1"]),
            gx=float(frame["gx1"]),
            gy=float(frame["gy1"]),
            gz=float(frame["gz1"]),
            dt_s=dt_s,
        )
        forearm = self._update_single(
            state=self.forearm,
            ax=float(frame["ax2"]),
            ay=float(frame["ay2"]),
            az=float(frame["az2"]),
            gx=float(frame["gx2"]),
            gy=float(frame["gy2"]),
            gz=float(frame["gz2"]),
            dt_s=dt_s,
        )

        rel_gx = hand["gx_dps"] - forearm["gx_dps"]
        rel_gy = hand["gy_dps"] - forearm["gy_dps"]
        rel_gz = hand["gz_dps"] - forearm["gz_dps"]
        rel_ax = hand["ax_g"] - forearm["ax_g"]
        rel_ay = hand["ay_g"] - forearm["ay_g"]
        rel_az = hand["az_g"] - forearm["az_g"]

        motion_energy = math.sqrt(rel_gx * rel_gx + rel_gy * rel_gy + rel_gz * rel_gz)

        return {
            "ts_ms": ts_ms,
            **{f"hand_{key}": value for key, value in hand.items()},
            **{f"forearm_{key}": value for key, value in forearm.items()},
            "rel_ax_g": rel_ax,
            "rel_ay_g": rel_ay,
            "rel_az_g": rel_az,
            "rel_gx_dps": rel_gx,
            "rel_gy_dps": rel_gy,
            "rel_gz_dps": rel_gz,
            "rel_roll_deg": hand["roll_deg"] - forearm["roll_deg"],
            "rel_pitch_deg": hand["pitch_deg"] - forearm["pitch_deg"],
            "rel_yaw_deg": hand["yaw_deg"] - forearm["yaw_deg"],
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
            "roll_deg": state.roll_deg,
            "pitch_deg": state.pitch_deg,
            "yaw_deg": state.yaw_deg,
        }


def enrich_stream(raw_data: pd.DataFrame) -> pd.DataFrame:
    if raw_data.empty:
        return pd.DataFrame()

    physical = convert_counts_to_physical(raw_data)
    processor = DualIMUStreamProcessor()
    rows: list[dict[str, float]] = []

    for frame in physical.to_dict(orient="records"):
        rows.append(processor.update(frame))

    return pd.DataFrame(rows)
