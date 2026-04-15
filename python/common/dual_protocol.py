from __future__ import annotations

from dataclasses import dataclass
from io import StringIO
from pathlib import Path
from typing import Iterable

import pandas as pd

FRAME_COLUMNS = [
    "ts_ms",
    "ax1",
    "ay1",
    "az1",
    "gx1",
    "gy1",
    "gz1",
    "ax2",
    "ay2",
    "az2",
    "gx2",
    "gy2",
    "gz2",
]

ACCEL_COLUMNS = ["ax1", "ay1", "az1", "ax2", "ay2", "az2"]
GYRO_COLUMNS = ["gx1", "gy1", "gz1", "gx2", "gy2", "gz2"]

ACCEL_LSB_PER_G = 8192.0
GYRO_LSB_PER_DPS = 65.5
DEFAULT_BAUD = 230400


@dataclass
class Recording:
    metadata: dict[str, str]
    data: pd.DataFrame


def parse_frame_line(line: str) -> dict[str, int] | None:
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return None

    parts = stripped.split(",")
    if len(parts) != len(FRAME_COLUMNS):
        return None

    try:
        values = [int(part) for part in parts]
    except ValueError:
        return None

    return dict(zip(FRAME_COLUMNS, values))


def dataframe_from_records(records: Iterable[dict[str, int]]) -> pd.DataFrame:
    df = pd.DataFrame.from_records(records, columns=FRAME_COLUMNS)
    if df.empty:
        return pd.DataFrame(columns=FRAME_COLUMNS)
    return df.astype({column: "int64" for column in FRAME_COLUMNS})


def load_recording(path: str | Path) -> Recording:
    metadata: dict[str, str] = {}
    lines: list[str] = []

    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("#"):
                content = line[1:].strip()
                if "=" in content:
                    key, value = content.split("=", 1)
                    metadata[key.strip()] = value.strip()
                continue
            lines.append(line)

    if not lines:
        return Recording(metadata=metadata, data=pd.DataFrame(columns=FRAME_COLUMNS))

    data = pd.read_csv(StringIO("".join(lines)))
    expected = set(FRAME_COLUMNS)
    if set(data.columns) != expected:
        raise ValueError(f"Unexpected columns in {path}: {list(data.columns)}")
    return Recording(metadata=metadata, data=data[FRAME_COLUMNS])


def save_recording(path: str | Path, data: pd.DataFrame, metadata: dict[str, str] | None = None) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    metadata = metadata or {}

    with path.open("w", encoding="utf-8", newline="") as handle:
        for key, value in metadata.items():
            handle.write(f"# {key}={value}\n")
        data.to_csv(handle, index=False)


def convert_counts_to_physical(data: pd.DataFrame) -> pd.DataFrame:
    physical = data.copy()
    for column in ACCEL_COLUMNS:
        physical[column] = physical[column] / ACCEL_LSB_PER_G
    for column in GYRO_COLUMNS:
        physical[column] = physical[column] / GYRO_LSB_PER_DPS
    return physical
