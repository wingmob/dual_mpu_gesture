from __future__ import annotations

import argparse
import time
from collections import deque

import joblib
import matplotlib.pyplot as plt
import pandas as pd
import serial

from features_single import MotionSegmenter, extract_feature_vector
from pose_single import SingleIMUStreamProcessor
from protocol_single import DEFAULT_BAUD, parse_frame_line


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Realtime single MPU6050 gesture recognition demo.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5.")
    parser.add_argument("--model", required=True, help="Path to the trained joblib model.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--plot", action="store_true", help="Show a lightweight realtime plot.")
    return parser


def init_plot():
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title("Single MPU Gesture Realtime Signals")
    ax.set_xlabel("Samples")
    ax.set_ylabel("Degrees / deg/s")
    roll_line, = ax.plot([], [], label="roll_deg")
    pitch_line, = ax.plot([], [], label="pitch_deg")
    energy_line, = ax.plot([], [], label="motion_energy")
    ax.legend(loc="upper right")
    return fig, ax, roll_line, pitch_line, energy_line


def update_plot(ax, roll_line, pitch_line, energy_line, history):
    if not history:
        return
    index = list(range(len(history)))
    roll_line.set_data(index, [sample["roll_deg"] for sample in history])
    pitch_line.set_data(index, [sample["pitch_deg"] for sample in history])
    energy_line.set_data(index, [sample["motion_energy"] for sample in history])
    ax.relim()
    ax.autoscale_view()
    plt.pause(0.001)


def main() -> None:
    args = build_parser().parse_args()
    bundle = joblib.load(args.model)
    model = bundle["model"]
    feature_names = bundle["feature_names"]

    processor = SingleIMUStreamProcessor()
    segmenter = MotionSegmenter()
    history: deque[dict[str, float]] = deque(maxlen=250)

    plot_handles = init_plot() if args.plot else None
    last_plot_time = time.perf_counter()

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print(f"Listening on {args.port} @ {args.baud}")
        while True:
            line = ser.readline().decode("utf-8", errors="ignore")
            frame = parse_frame_line(line)
            if frame is None:
                continue

            enriched = processor.update(frame)
            history.append(enriched)
            segment = segmenter.push(enriched)

            if segment is not None:
                feature_vector = extract_feature_vector(segment)
                feature_row = pd.DataFrame([[feature_vector[name] for name in feature_names]], columns=feature_names)
                probabilities = model.predict_proba(feature_row)[0]
                label = model.classes_[probabilities.argmax()]
                confidence = probabilities.max()
                print(
                    f"[gesture] label={label:>18s} confidence={confidence:.3f} "
                    f"duration={feature_vector['duration_s']:.2f}s peak={feature_vector['motion_energy_max']:.1f}"
                )

            if plot_handles and time.perf_counter() - last_plot_time >= 0.05:
                _, ax, roll_line, pitch_line, energy_line = plot_handles
                update_plot(ax, roll_line, pitch_line, energy_line, history)
                last_plot_time = time.perf_counter()


if __name__ == "__main__":
    main()
