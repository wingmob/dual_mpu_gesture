from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import serial

from protocol import DEFAULT_BAUD, dataframe_from_records, parse_frame_line, save_recording


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record labeled gesture trials from the dual MPU stream.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5.")
    parser.add_argument("--label", required=True, help="Gesture label, for example flexion.")
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--seconds", type=float, default=2.5, help="Recording time per trial.")
    parser.add_argument("--prep-seconds", type=float, default=2.0, help="Countdown before recording each trial.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--out-dir", type=Path, default=Path("data/raw"))
    parser.add_argument("--subject", default="subject01")
    return parser


def countdown(seconds: float) -> None:
    remaining = int(seconds)
    while remaining > 0:
        print(f"  start in {remaining}...")
        time.sleep(1.0)
        remaining -= 1


def record_trial(ser: serial.Serial, seconds: float) -> list[dict[str, int]]:
    records: list[dict[str, int]] = []
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore")
        frame = parse_frame_line(line)
        if frame is not None:
            records.append(frame)
    return records


def main() -> None:
    args = build_parser().parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print(f"Connected to {args.port} @ {args.baud}")
        print("Keep both sensors still for 2 seconds after reset or after sending recalibration.")
        time.sleep(2.0)

        for trial_idx in range(1, args.trials + 1):
            print(f"\nTrial {trial_idx}/{args.trials} for label '{args.label}'")
            countdown(args.prep_seconds)
            print("  perform gesture now")
            records = record_trial(ser, args.seconds)
            data = dataframe_from_records(records)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{args.subject}_{args.label}_{trial_idx:02d}_{timestamp}.csv"
            path = args.out_dir / filename
            metadata = {
                "label": args.label,
                "trial": str(trial_idx),
                "subject": args.subject,
                "captured_at": timestamp,
            }
            save_recording(path, data, metadata)
            print(f"  saved {len(data)} samples to {path}")
            time.sleep(0.8)


if __name__ == "__main__":
    main()
