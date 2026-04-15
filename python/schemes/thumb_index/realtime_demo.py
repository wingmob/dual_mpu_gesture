from __future__ import annotations

import argparse
from collections import deque
import os
import sys
import time
from dataclasses import dataclass, field

import joblib
import pandas as pd
import serial

from .features import MotionSegmenter, extract_feature_vector, select_peak_window
from .pose import ThumbIndexStreamProcessor
from ...common.dual_protocol import DEFAULT_BAUD, convert_counts_to_physical, parse_frame_line

GESTURE_DISPLAY_ORDER = [
    "fist",
    "open_palm",
    "thumb_up",
    "victory",
    "point",
    "ok",
    "flexion",
    "extension",
    "radial_deviation",
    "ulnar_deviation",
    "pronation",
    "supination",
]


@dataclass
class DashboardState:
    phase: str = "waiting"
    motion_energy: float = 0.0
    last_label: str | None = None
    last_confidence: float = 0.0
    last_duration_s: float = 0.0
    last_peak_motion: float = 0.0
    last_result_kind: str = "waiting"
    last_result_time: float | None = None
    probabilities: dict[str, float] = field(default_factory=dict)


class TerminalDashboard:
    def __init__(self) -> None:
        self._enabled = sys.stdout.isatty()
        self._first_render = True
        if self._enabled:
            self._enable_virtual_terminal()

    @property
    def enabled(self) -> bool:
        return self._enabled

    def render(self, text: str) -> None:
        if not self._enabled:
            return
        prefix = "\x1b[2J\x1b[H" if self._first_render else "\x1b[H\x1b[J"
        sys.stdout.write(prefix)
        sys.stdout.write(text)
        if not text.endswith("\n"):
            sys.stdout.write("\n")
        sys.stdout.flush()
        self._first_render = False

    def close(self) -> None:
        if self._enabled and not self._first_render:
            sys.stdout.write("\n")
            sys.stdout.flush()

    @staticmethod
    def _enable_virtual_terminal() -> None:
        if os.name != "nt":
            return
        try:
            import ctypes

            handle = ctypes.windll.kernel32.GetStdHandle(-11)
            if handle in (0, -1):
                return
            mode = ctypes.c_uint()
            if ctypes.windll.kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
                ctypes.windll.kernel32.SetConsoleMode(handle, mode.value | 0x0004)
        except Exception:
            return


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Realtime thumb/index dual MPU6050 gesture recognition demo.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5.")
    parser.add_argument("--model", required=True, help="Path to the trained joblib model.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--min-confidence", type=float, default=0.55, help="Suppress low-confidence predictions.")
    parser.add_argument("--start-threshold", type=float, default=18.0, help="Motion energy threshold to start a segment.")
    parser.add_argument("--stop-threshold", type=float, default=7.0, help="Motion energy threshold to count quiet frames.")
    parser.add_argument("--quiet-frames", type=int, default=14, help="How many quiet frames end a segment.")
    parser.add_argument("--min-frames", type=int, default=15, help="Minimum segment length to classify.")
    parser.add_argument("--max-frames", type=int, default=200, help="Maximum segment length before forced classification.")
    parser.add_argument("--min-duration", type=float, default=0.20, help="Ignore segments shorter than this duration in seconds.")
    parser.add_argument("--min-peak", type=float, default=35.0, help="Ignore segments whose motion peak is below this value.")
    parser.add_argument("--cooldown-seconds", type=float, default=0.60, help="Ignore new segments for a short time after each emitted result.")
    return parser


def order_labels(classes: list[str]) -> list[str]:
    ordered = [label for label in GESTURE_DISPLAY_ORDER if label in classes]
    ordered.extend(label for label in classes if label not in ordered)
    return ordered


def make_probability_bar(value: float, width: int = 28) -> str:
    clamped = max(0.0, min(1.0, float(value)))
    filled = int(round(clamped * width))
    return f"[{'#' * filled}{'.' * (width - filled)}]"


def format_result_summary(state: DashboardState) -> str:
    if state.last_result_time is None or state.last_label is None:
        return "waiting for first completed gesture segment"

    age_s = time.perf_counter() - state.last_result_time
    return (
        f"{state.last_result_kind}: {state.last_label}  conf={state.last_confidence:.3f}  "
        f"duration={state.last_duration_s:.2f}s  peak={state.last_peak_motion:.1f}  "
        f"age={age_s:.1f}s"
    )


def render_dashboard(
    state: DashboardState,
    label_order: list[str],
    port: str,
    baud: int,
) -> str:
    lines = [
        "Thumb + Index Dual MPU Gesture Realtime",
        f"Port: {port} @ {baud}",
        f"State: {state.phase:<10s} motion_energy={state.motion_energy:7.1f}",
        f"Last result: {format_result_summary(state)}",
        "",
        "Expected motion: neutral hand -> target gesture -> hold briefly -> relax.",
        "",
        "Gesture probabilities:",
    ]

    best_label = None
    best_score = -1.0
    if state.probabilities:
        best_label, best_score = max(state.probabilities.items(), key=lambda item: item[1])

    for label in label_order:
        score = float(state.probabilities.get(label, 0.0))
        marker = ">" if label == best_label and best_score >= 0.0 else " "
        lines.append(f"{marker} {label:<17s} {make_probability_bar(score)} {score:0.3f}")

    lines.extend(
        [
            "",
            "Legend: 'accepted' means confidence passed the threshold, 'uncertain' means it did not.",
            "Press Ctrl+C to stop.",
        ]
    )
    return "\n".join(lines)


def wait_for_stream_ready(ser: serial.Serial, timeout_s: float = 12.0) -> dict[str, int] | None:
    print("Waiting for board reset and gyro calibration (~6s)...")
    deadline = time.perf_counter() + timeout_s
    recent_lines: deque[str] = deque(maxlen=20)
    while time.perf_counter() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore")
        if not line:
            continue
        stripped = line.strip()
        if stripped:
            recent_lines.append(stripped)
        frame = parse_frame_line(line)
        if frame is not None:
            print("Stream ready. The terminal dashboard will update in place.")
            return frame
        if stripped.startswith("# startup_halted"):
            details = "\n".join(recent_lines)
            raise RuntimeError(
                "Firmware reported startup_halted. Recent serial output:\n"
                f"{details}"
            )
    return None


def resolve_phase(segmenter: MotionSegmenter, last_emit_time: float, cooldown_seconds: float) -> str:
    if getattr(segmenter, "_active", False):
        return "capturing"
    cooldown_left = cooldown_seconds - max(0.0, time.perf_counter() - last_emit_time)
    if cooldown_left > 0:
        return f"cooldown {cooldown_left:0.2f}s"
    return "idle"


def main() -> None:
    args = build_parser().parse_args()
    bundle = joblib.load(args.model)
    model = bundle["model"]
    feature_names = bundle["feature_names"]
    label_order = order_labels([str(label) for label in model.classes_])

    processor = ThumbIndexStreamProcessor()
    segmenter = MotionSegmenter(
        start_threshold_dps=args.start_threshold,
        stop_threshold_dps=args.stop_threshold,
        quiet_frames=args.quiet_frames,
        min_frames=args.min_frames,
        max_frames=args.max_frames,
    )

    state = DashboardState(probabilities={label: 0.0 for label in label_order})
    dashboard = TerminalDashboard()
    last_dashboard_time = 0.0
    last_emit_time = -1e9

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
            print(f"Listening on {args.port} @ {args.baud}")
            first_frame = wait_for_stream_ready(ser)
            if first_frame is None:
                raise TimeoutError("Timed out waiting for CSV frames from the board.")

            dashboard.render(render_dashboard(state, label_order, args.port, args.baud))
            last_dashboard_time = time.perf_counter()

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
                segment = segmenter.push(enriched)

                state.motion_energy = float(enriched["motion_energy"])
                state.phase = resolve_phase(segmenter, last_emit_time, args.cooldown_seconds)

                dashboard_should_refresh = False
                if segment is not None:
                    trimmed_segment = select_peak_window(segment)
                    feature_vector = extract_feature_vector(trimmed_segment)
                    duration_s = float(feature_vector["duration_s"])
                    peak_motion = float(feature_vector["motion_energy_max"])
                    now = time.perf_counter()

                    if (
                        duration_s >= args.min_duration
                        and peak_motion >= args.min_peak
                        and now - last_emit_time >= args.cooldown_seconds
                    ):
                        feature_row = pd.DataFrame(
                            [[feature_vector[name] for name in feature_names]],
                            columns=feature_names,
                        )
                        probabilities = model.predict_proba(feature_row)[0]
                        label = str(model.classes_[probabilities.argmax()])
                        confidence = float(probabilities.max())

                        state.probabilities = {
                            str(class_label): float(probabilities[idx])
                            for idx, class_label in enumerate(model.classes_)
                        }
                        state.last_label = label
                        state.last_confidence = confidence
                        state.last_duration_s = duration_s
                        state.last_peak_motion = peak_motion
                        state.last_result_kind = "accepted" if confidence >= args.min_confidence else "uncertain"
                        state.last_result_time = now

                        if not dashboard.enabled:
                            if confidence < args.min_confidence:
                                print(
                                    f"[gesture?] uncertain best={label:>18s} confidence={confidence:.3f} "
                                    f"duration={duration_s:.2f}s peak={peak_motion:.1f}"
                                )
                            else:
                                print(
                                    f"[gesture] label={label:>18s} confidence={confidence:.3f} "
                                    f"duration={duration_s:.2f}s peak={peak_motion:.1f}"
                                )

                        last_emit_time = now
                        state.phase = resolve_phase(segmenter, last_emit_time, args.cooldown_seconds)
                        dashboard_should_refresh = True

                if dashboard_should_refresh or time.perf_counter() - last_dashboard_time >= 0.10:
                    dashboard.render(render_dashboard(state, label_order, args.port, args.baud))
                    last_dashboard_time = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.close()


if __name__ == "__main__":
    main()
