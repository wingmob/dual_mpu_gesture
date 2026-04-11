from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import time

import joblib
import pandas as pd
import serial

from camera_gestures import CameraGestureEngine, ensure_camera_dependencies
from features import MotionSegmenter, extract_feature_vector, select_peak_window
from pose import DualIMUStreamProcessor
from protocol import DEFAULT_BAUD, convert_counts_to_physical, parse_frame_line


@dataclass
class DynamicGestureEvent:
    label: str
    confidence: float
    peak_motion: float
    duration_s: float
    detected_at: float


class DynamicGestureEngine:
    def __init__(
        self,
        model_path: str,
        min_confidence: float = 0.55,
        start_threshold: float = 45.0,
        stop_threshold: float = 20.0,
        quiet_frames: int = 6,
        min_frames: int = 12,
        max_frames: int = 120,
        min_duration: float = 0.25,
        min_peak: float = 120.0,
        cooldown_seconds: float = 0.60,
    ) -> None:
        bundle = joblib.load(model_path)
        self._model = bundle["model"]
        self._feature_names = bundle["feature_names"]
        self._processor = DualIMUStreamProcessor()
        self._segmenter = MotionSegmenter(
            start_threshold_dps=start_threshold,
            stop_threshold_dps=stop_threshold,
            quiet_frames=quiet_frames,
            min_frames=min_frames,
            max_frames=max_frames,
        )
        self._min_confidence = min_confidence
        self._min_duration = min_duration
        self._min_peak = min_peak
        self._cooldown_seconds = cooldown_seconds
        self._last_event: DynamicGestureEvent | None = None
        self._last_emit_time = -1e9

    @property
    def last_event(self) -> DynamicGestureEvent | None:
        return self._last_event

    def push_line(self, line: str) -> DynamicGestureEvent | None:
        frame = parse_frame_line(line)
        if frame is None:
            return None

        physical_frame = convert_counts_to_physical(pd.DataFrame([frame])).to_dict(orient="records")[0]
        enriched = self._processor.update(physical_frame)
        segment = self._segmenter.push(enriched)
        if segment is None:
            return None

        trimmed_segment = select_peak_window(segment)
        feature_vector = extract_feature_vector(trimmed_segment)
        duration_s = float(feature_vector["duration_s"])
        peak_motion = float(feature_vector["motion_energy_max"])
        now = time.perf_counter()
        if duration_s < self._min_duration or peak_motion < self._min_peak or now - self._last_emit_time < self._cooldown_seconds:
            return None
        feature_row = pd.DataFrame([[feature_vector[name] for name in self._feature_names]], columns=self._feature_names)
        probabilities = self._model.predict_proba(feature_row)[0]
        label = self._model.classes_[probabilities.argmax()]
        confidence = float(probabilities.max())
        if confidence < self._min_confidence:
            self._last_emit_time = now
            return None

        event = DynamicGestureEvent(
            label=label,
            confidence=confidence,
            peak_motion=peak_motion,
            duration_s=duration_s,
            detected_at=now,
        )
        self._last_event = event
        self._last_emit_time = now
        return event


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Hybrid realtime gesture demo: dual MPU for wrist motions + camera for finger gestures."
    )
    parser.add_argument("--port", help="Serial port for the dual MPU stream, for example COM5.")
    parser.add_argument("--dynamic-model", help="Path to the trained dual-MPU joblib model.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--no-camera", action="store_true", help="Disable camera gesture recognition.")
    parser.add_argument("--dynamic-hold-seconds", type=float, default=1.2)
    parser.add_argument("--min-dynamic-confidence", type=float, default=0.55)
    parser.add_argument("--start-threshold", type=float, default=45.0, help="Motion energy threshold to start a dynamic segment.")
    parser.add_argument("--stop-threshold", type=float, default=20.0, help="Motion energy threshold to count quiet frames.")
    parser.add_argument("--quiet-frames", type=int, default=6, help="How many quiet frames end a dynamic segment.")
    parser.add_argument("--min-frames", type=int, default=12, help="Minimum dynamic segment length to classify.")
    parser.add_argument("--max-frames", type=int, default=120, help="Maximum dynamic segment length before forced classification.")
    parser.add_argument("--min-duration", type=float, default=0.25, help="Ignore dynamic segments shorter than this duration in seconds.")
    parser.add_argument("--min-peak", type=float, default=120.0, help="Ignore dynamic segments whose motion peak is below this value.")
    parser.add_argument("--cooldown-seconds", type=float, default=0.60, help="Ignore new dynamic segments for a short time after each emitted result.")
    parser.add_argument(
        "--max-serial-lines-per-cycle",
        type=int,
        default=6,
        help="Limit how many serial lines are processed before each camera refresh.",
    )
    return parser


def resolve_active_label(
    dynamic_engine: DynamicGestureEngine | None,
    camera_label: str | None,
    dynamic_hold_seconds: float,
) -> str | None:
    if dynamic_engine and dynamic_engine.last_event:
        age_s = time.perf_counter() - dynamic_engine.last_event.detected_at
        if age_s <= dynamic_hold_seconds:
            return dynamic_engine.last_event.label
    return camera_label


def wait_for_stream_ready(ser: serial.Serial, timeout_s: float = 12.0) -> str | None:
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
        if parse_frame_line(line) is not None:
            print("Dual MPU stream ready.")
            return line
        if stripped.startswith("# startup_halted"):
            details = "\n".join(recent_lines)
            raise RuntimeError(
                "Firmware reported startup_halted. Recent serial output:\n"
                f"{details}"
            )
    return None


def main() -> None:
    args = build_parser().parse_args()

    use_dynamic = bool(args.port and args.dynamic_model)
    use_camera = not args.no_camera
    if not use_dynamic and not use_camera:
        raise ValueError("Enable at least one modality: provide --port/--dynamic-model and/or keep camera enabled.")
    if bool(args.port) != bool(args.dynamic_model):
        raise ValueError("--port and --dynamic-model must be provided together.")

    dynamic_engine = (
        DynamicGestureEngine(
            args.dynamic_model,
            args.min_dynamic_confidence,
            args.start_threshold,
            args.stop_threshold,
            args.quiet_frames,
            args.min_frames,
            args.max_frames,
            args.min_duration,
            args.min_peak,
            args.cooldown_seconds,
        )
        if use_dynamic
        else None
    )

    camera_engine: CameraGestureEngine | None = None
    if use_camera:
        ensure_camera_dependencies()
        camera_engine = CameraGestureEngine(camera_index=args.camera_index)

    serial_port: serial.Serial | None = None
    first_dynamic_line: str | None = None
    if use_dynamic:
        serial_port = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"Listening to dual MPU stream on {args.port} @ {args.baud}")
        first_dynamic_line = wait_for_stream_ready(serial_port)
        if first_dynamic_line is None:
            raise TimeoutError("Timed out waiting for CSV frames from the board.")
    if camera_engine:
        print(f"Listening to camera index {args.camera_index}")

    last_camera_label: str | None = None
    last_active_label: str | None = None

    try:
        while True:
            if serial_port and dynamic_engine:
                serial_lines_processed = 0
                while serial_lines_processed < max(1, args.max_serial_lines_per_cycle):
                    if first_dynamic_line is not None:
                        raw_line = first_dynamic_line
                        first_dynamic_line = None
                    else:
                        raw_line = serial_port.readline().decode("utf-8", errors="ignore")
                    if not raw_line:
                        break
                    serial_lines_processed += 1
                    event = dynamic_engine.push_line(raw_line)
                    if event is not None:
                        print(
                            f"[dynamic] label={event.label:>18s} confidence={event.confidence:.3f} "
                            f"duration={event.duration_s:.2f}s peak={event.peak_motion:.1f}"
                        )

            camera_prediction_label: str | None = None
            if camera_engine:
                frame, prediction = camera_engine.read()
                camera_prediction_label = prediction.stable_label
                if prediction.stable_label != last_camera_label and prediction.stable_label is not None:
                    print(f"[camera] label={prediction.stable_label:>18s} confidence={prediction.confidence:.3f}")
                    last_camera_label = prediction.stable_label

                active_label = resolve_active_label(dynamic_engine, camera_prediction_label, args.dynamic_hold_seconds)
                if active_label != last_active_label and active_label is not None:
                    print(f"[active] label={active_label}")
                    last_active_label = active_label

                frame = camera_engine.draw_overlay(frame, prediction, active_label)
                from camera_gestures import cv2  # Localized import keeps dependency handling in one module.

                cv2.imshow("Hybrid Gesture Demo", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
            else:
                active_label = resolve_active_label(dynamic_engine, None, args.dynamic_hold_seconds)
                if active_label != last_active_label and active_label is not None:
                    print(f"[active] label={active_label}")
                    last_active_label = active_label
                time.sleep(0.01)
    finally:
        if serial_port is not None:
            serial_port.close()
        if camera_engine is not None:
            from camera_gestures import cv2

            camera_engine.close()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
