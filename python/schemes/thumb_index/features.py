from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from .pose import enrich_stream

FEATURE_SIGNAL_COLUMNS = [
    "thumb_ax_g",
    "thumb_ay_g",
    "thumb_az_g",
    "thumb_gx_dps",
    "thumb_gy_dps",
    "thumb_gz_dps",
    "thumb_roll_deg",
    "thumb_pitch_deg",
    "thumb_yaw_deg",
    "thumb_accel_norm_g",
    "thumb_gyro_norm_dps",
    "index_ax_g",
    "index_ay_g",
    "index_az_g",
    "index_gx_dps",
    "index_gy_dps",
    "index_gz_dps",
    "index_roll_deg",
    "index_pitch_deg",
    "index_yaw_deg",
    "index_accel_norm_g",
    "index_gyro_norm_dps",
    "rel_ax_g",
    "rel_ay_g",
    "rel_az_g",
    "rel_gravity_ax_g",
    "rel_gravity_ay_g",
    "rel_gravity_az_g",
    "rel_gx_dps",
    "rel_gy_dps",
    "rel_gz_dps",
    "rel_roll_deg",
    "rel_pitch_deg",
    "rel_yaw_deg",
    "gravity_angle_deg",
    "pair_gyro_mean_dps",
    "pair_accel_mean_g",
    "pose_gap_deg",
    "relative_motion_energy",
    "motion_energy",
]

POSE_STATE_COLUMNS = [
    "thumb_roll_deg",
    "thumb_pitch_deg",
    "index_roll_deg",
    "index_pitch_deg",
    "rel_roll_deg",
    "rel_pitch_deg",
    "rel_yaw_deg",
    "gravity_angle_deg",
    "pose_gap_deg",
]

SUMMARY_FUNCS = {
    "mean": np.mean,
    "std": np.std,
    "min": np.min,
    "max": np.max,
    "rms": lambda arr: float(np.sqrt(np.mean(np.square(arr)))),
    "delta": lambda arr: float(arr[-1] - arr[0]),
}


def select_peak_window(raw_data: pd.DataFrame, window_samples: int = 150) -> pd.DataFrame:
    if raw_data.empty or len(raw_data) <= window_samples:
        return raw_data.reset_index(drop=True)

    if "motion_energy" in raw_data.columns:
        enriched = raw_data.reset_index(drop=True)
    else:
        enriched = enrich_stream(raw_data)
    scores = (
        enriched["motion_energy"]
        .rolling(window=11, min_periods=1, center=True)
        .mean()
        .to_numpy()
    )
    peak_idx = int(np.argmax(scores))
    start = max(0, peak_idx - window_samples // 3)
    end = min(len(raw_data), start + window_samples)
    start = max(0, end - window_samples)
    return raw_data.iloc[start:end].reset_index(drop=True)


def _window_mean(values: pd.Series) -> float:
    return float(values.to_numpy(dtype=float).mean())


def _window_std(values: pd.Series) -> float:
    return float(values.to_numpy(dtype=float).std())


def extract_feature_vector(raw_or_enriched: pd.DataFrame) -> dict[str, float]:
    if raw_or_enriched.empty:
        raise ValueError("Cannot extract features from an empty dataframe.")

    if "motion_energy" in raw_or_enriched.columns:
        enriched = raw_or_enriched.reset_index(drop=True)
    else:
        enriched = enrich_stream(raw_or_enriched)

    features: dict[str, float] = {}
    duration_s = max(0.0, (float(enriched["ts_ms"].iloc[-1]) - float(enriched["ts_ms"].iloc[0])) / 1000.0)
    features["duration_s"] = duration_s
    features["sample_count"] = float(len(enriched))
    features["energy_peak_idx_ratio"] = float(np.argmax(enriched["motion_energy"].to_numpy()) / max(1, len(enriched) - 1))

    for column in FEATURE_SIGNAL_COLUMNS:
        values = enriched[column].to_numpy(dtype=float)
        for stat_name, func in SUMMARY_FUNCS.items():
            features[f"{column}_{stat_name}"] = float(func(values))

    head_size = max(5, len(enriched) // 5)
    tail_size = max(5, len(enriched) // 5)
    head = enriched.head(head_size)
    tail = enriched.tail(tail_size)

    for column in POSE_STATE_COLUMNS:
        head_mean = _window_mean(head[column])
        tail_mean = _window_mean(tail[column])
        features[f"{column}_head_mean"] = head_mean
        features[f"{column}_tail_mean"] = tail_mean
        features[f"{column}_tail_std"] = _window_std(tail[column])
        features[f"{column}_head_to_tail_delta"] = tail_mean - head_mean

    features["tail_motion_energy_mean"] = _window_mean(tail["motion_energy"])
    features["tail_relative_motion_energy_mean"] = _window_mean(tail["relative_motion_energy"])
    features["tail_pair_gyro_mean_dps_mean"] = _window_mean(tail["pair_gyro_mean_dps"])

    return features


@dataclass
class MotionSegmenter:
    start_threshold_dps: float = 18.0
    stop_threshold_dps: float = 7.0
    quiet_frames: int = 14
    min_frames: int = 15
    max_frames: int = 200
    pre_roll: int = 10
    _pre_buffer: deque[dict[str, float]] = field(init=False)
    _active_frames: list[dict[str, float]] = field(default_factory=list)
    _quiet_counter: int = 0
    _active: bool = False

    def __post_init__(self) -> None:
        self._pre_buffer = deque(maxlen=self.pre_roll)

    def push(self, sample: dict[str, float]) -> pd.DataFrame | None:
        energy = float(sample["motion_energy"])
        if not self._active:
            self._pre_buffer.append(sample)
            if energy >= self.start_threshold_dps:
                self._active = True
                self._active_frames = list(self._pre_buffer)
                self._quiet_counter = 0
            return None

        self._active_frames.append(sample)
        if energy <= self.stop_threshold_dps:
            self._quiet_counter += 1
        else:
            self._quiet_counter = 0

        if len(self._active_frames) >= self.max_frames or self._quiet_counter >= self.quiet_frames:
            segment = pd.DataFrame(self._active_frames[:-self._quiet_counter or None])
            self._reset()
            if len(segment) >= self.min_frames:
                return segment.reset_index(drop=True)
        return None

    def _reset(self) -> None:
        self._pre_buffer.clear()
        self._active_frames = []
        self._quiet_counter = 0
        self._active = False
