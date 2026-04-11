from __future__ import annotations

import argparse
import time
from collections import deque

import joblib
import matplotlib.pyplot as plt
import pandas as pd
import serial

from features import MotionSegmenter, extract_feature_vector, select_peak_window
from pose import DualIMUStreamProcessor
from protocol import DEFAULT_BAUD, convert_counts_to_physical, parse_frame_line


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Realtime dual MPU6050 gesture recognition demo.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5.")
    parser.add_argument("--model", required=True, help="Path to the trained joblib model.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--plot", action="store_true", help="Show a lightweight realtime plot.")
    parser.add_argument("--min-confidence", type=float, default=0.60, help="Suppress low-confidence predictions.")
    parser.add_argument("--start-threshold", type=float, default=45.0, help="Motion energy threshold to start a segment.")
    parser.add_argument("--stop-threshold", type=float, default=20.0, help="Motion energy threshold to count quiet frames.")
    parser.add_argument("--quiet-frames", type=int, default=6, help="How many quiet frames end a segment.")
    parser.add_argument("--min-frames", type=int, default=12, help="Minimum segment length to classify.")
    parser.add_argument("--max-frames", type=int, default=120, help="Maximum segment length before forced classification.")
    parser.add_argument("--min-duration", type=float, default=0.25, help="Ignore segments shorter than this duration in seconds.")
    parser.add_argument("--min-peak", type=float, default=120.0, help="Ignore segments whose motion peak is below this value.")
    parser.add_argument("--cooldown-seconds", type=float, default=0.60, help="Ignore new segments for a short time after each emitted result.")
    return parser


def init_plot():
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title("Dual MPU Gesture Realtime Signals")
    ax.set_xlabel("Samples")
    ax.set_ylabel("Degrees / deg/s")
    rel_roll_line, = ax.plot([], [], label="rel_roll_deg")
    rel_pitch_line, = ax.plot([], [], label="rel_pitch_deg")
    energy_line, = ax.plot([], [], label="motion_energy")
    ax.legend(loc="upper right")
    return fig, ax, rel_roll_line, rel_pitch_line, energy_line


def update_plot(ax, rel_roll_line, rel_pitch_line, energy_line, history):
    if not history:
        return
    index = list(range(len(history)))
    rel_roll_line.set_data(index, [sample["rel_roll_deg"] for sample in history])
    rel_pitch_line.set_data(index, [sample["rel_pitch_deg"] for sample in history])
    energy_line.set_data(index, [sample["motion_energy"] for sample in history])
    ax.relim()
    ax.autoscale_view()
    plt.pause(0.001)


def wait_for_stream_ready(ser: serial.Serial, timeout_s: float = 12.0) -> dict[str, int] | None:
    print("Waiting for board reset and gyro calibration (~6s)...")
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore")
        if not line:
            continue
        frame = parse_frame_line(line)
        if frame is not None:
            print("Stream ready. Perform one clear gesture, then return to neutral.")
            return frame
        stripped = line.strip()
        if stripped.startswith("# startup_halted"):
            raise RuntimeError("Firmware reported startup_halted. Check the serial log with capture_stream.py.")
    return None


def main() -> None:
    args = build_parser().parse_args()
    bundle = joblib.load(args.model)
    model = bundle["model"]
    feature_names = bundle["feature_names"]

    processor = DualIMUStreamProcessor()
    segmenter = MotionSegmenter(
        start_threshold_dps=args.start_threshold,
        stop_threshold_dps=args.stop_threshold,
        quiet_frames=args.quiet_frames,
        min_frames=args.min_frames,
        max_frames=args.max_frames,
    )
    history: deque[dict[str, float]] = deque(maxlen=250)

    plot_handles = init_plot() if args.plot else None
    last_plot_time = time.perf_counter()
    last_emit_time = -1e9

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        print(f"Listening on {args.port} @ {args.baud}")
        first_frame = wait_for_stream_ready(ser)
        if first_frame is None:
            raise TimeoutError("Timed out waiting for CSV frames from the board.")
        while True:
            if first_frame is not None:
                frame = first_frame
                first_frame = None
            else:
                line = ser.readline().decode("utf-8", errors="ignore")
                frame = parse_frame_line(line)
                if frame is None:
                    continue

            physical_frame = convert_counts_to_physical(pd.DataFrame([frame])).to_dict(orient="records")[0]
            enriched = processor.update(physical_frame)
            history.append(enriched)
            segment = segmenter.push(enriched)

            if segment is not None:
                trimmed_segment = select_peak_window(segment)
                feature_vector = extract_feature_vector(trimmed_segment)
                duration_s = float(feature_vector["duration_s"])
                peak_motion = float(feature_vector["motion_energy_max"])
                now = time.perf_counter()
                if duration_s < args.min_duration or peak_motion < args.min_peak or now - last_emit_time < args.cooldown_seconds:
                    continue
                feature_row = pd.DataFrame([[feature_vector[name] for name in feature_names]], columns=feature_names)
                probabilities = model.predict_proba(feature_row)[0]
                label = model.classes_[probabilities.argmax()]
                confidence = probabilities.max()
                if confidence < args.min_confidence:
                    print(
                        f"[gesture?] uncertain best={label:>18s} confidence={confidence:.3f} "
                        f"duration={duration_s:.2f}s peak={peak_motion:.1f}"
                    )
                    last_emit_time = now
                    continue
                print(
                    f"[gesture] label={label:>18s} confidence={confidence:.3f} "
                    f"duration={duration_s:.2f}s peak={peak_motion:.1f}"
                )
                last_emit_time = now

            if plot_handles and time.perf_counter() - last_plot_time >= 0.05:
                _, ax, rel_roll_line, rel_pitch_line, energy_line = plot_handles
                update_plot(ax, rel_roll_line, rel_pitch_line, energy_line, history)
                last_plot_time = time.perf_counter()


if __name__ == "__main__":
    main()
