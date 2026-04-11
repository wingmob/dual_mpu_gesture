from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
import math
from typing import Any

try:
    import cv2
except ImportError:  # pragma: no cover - exercised at runtime when dependency is missing
    cv2 = None

try:
    import mediapipe as mp
except ImportError:  # pragma: no cover - exercised at runtime when dependency is missing
    mp = None


FINGER_GESTURE_LABELS = [
    "fist",
    "open_palm",
    "thumb_up",
    "victory",
    "point",
    "ok",
]


@dataclass
class CameraGesturePrediction:
    raw_label: str | None
    stable_label: str | None
    confidence: float
    handedness: str | None
    debug_text: str = ""


@dataclass
class TemporalLabelSmoother:
    history_len: int = 7
    min_consensus: int = 4
    _history: deque[str | None] = field(init=False)

    def __post_init__(self) -> None:
        self._history = deque(maxlen=self.history_len)

    def push(self, label: str | None) -> str | None:
        self._history.append(label)
        votes = [item for item in self._history if item]
        if not votes:
            return None
        counts = Counter(votes)
        best_label, best_count = counts.most_common(1)[0]
        if best_count >= self.min_consensus:
            return best_label
        return None


def ensure_camera_dependencies() -> None:
    missing: list[str] = []
    if cv2 is None:
        missing.append("opencv-python")
    if mp is None:
        missing.append("mediapipe")
    if missing:
        joined = ", ".join(missing)
        raise RuntimeError(
            f"Missing camera dependencies: {joined}. "
            "Install them in a Python 3.11/3.12 environment before using camera gestures."
        )


def _xyz(landmarks: Any, index: int) -> tuple[float, float, float]:
    point = landmarks.landmark[index]
    return float(point.x), float(point.y), float(point.z)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _angle_deg(a: tuple[float, float, float], b: tuple[float, float, float], c: tuple[float, float, float]) -> float:
    ab = (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    cb = (c[0] - b[0], c[1] - b[1], c[2] - b[2])
    ab_norm = math.sqrt(ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2)
    cb_norm = math.sqrt(cb[0] ** 2 + cb[1] ** 2 + cb[2] ** 2)
    if ab_norm <= 1e-6 or cb_norm <= 1e-6:
        return 0.0
    cosine = max(-1.0, min(1.0, (ab[0] * cb[0] + ab[1] * cb[1] + ab[2] * cb[2]) / (ab_norm * cb_norm)))
    return math.degrees(math.acos(cosine))


def _palm_size(landmarks: Any) -> float:
    wrist = _xyz(landmarks, 0)
    index_mcp = _xyz(landmarks, 5)
    pinky_mcp = _xyz(landmarks, 17)
    return max(1e-6, (_distance(wrist, index_mcp) + _distance(wrist, pinky_mcp) + _distance(index_mcp, pinky_mcp)) / 3.0)


def _palm_center(landmarks: Any) -> tuple[float, float, float]:
    indices = [0, 5, 9, 13, 17]
    points = [_xyz(landmarks, index) for index in indices]
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
        sum(point[2] for point in points) / len(points),
    )


def _finger_extended(landmarks: Any, mcp_idx: int, pip_idx: int, dip_idx: int, tip_idx: int) -> bool:
    wrist = _xyz(landmarks, 0)
    mcp = _xyz(landmarks, mcp_idx)
    pip = _xyz(landmarks, pip_idx)
    dip = _xyz(landmarks, dip_idx)
    tip = _xyz(landmarks, tip_idx)
    pip_angle = _angle_deg(mcp, pip, dip)
    dip_angle = _angle_deg(pip, dip, tip)
    tip_distance = _distance(tip, wrist)
    pip_distance = _distance(pip, wrist)
    return pip_angle > 150.0 and dip_angle > 150.0 and tip_distance > pip_distance * 1.08


def _thumb_extended(landmarks: Any) -> bool:
    palm_center = _palm_center(landmarks)
    cmc = _xyz(landmarks, 1)
    mcp = _xyz(landmarks, 2)
    ip = _xyz(landmarks, 3)
    tip = _xyz(landmarks, 4)
    mcp_angle = _angle_deg(cmc, mcp, ip)
    ip_angle = _angle_deg(mcp, ip, tip)
    tip_distance = _distance(tip, palm_center)
    ip_distance = _distance(ip, palm_center)
    return mcp_angle > 135.0 and ip_angle > 150.0 and tip_distance > ip_distance * 1.08


def classify_finger_gesture(landmarks: Any, handedness: str | None) -> tuple[str | None, float, str]:
    del handedness  # Handedness is available for future tuning, but the current heuristic stays orientation-light.

    palm_size = _palm_size(landmarks)
    wrist = _xyz(landmarks, 0)
    thumb_tip = _xyz(landmarks, 4)
    thumb_ip = _xyz(landmarks, 3)
    index_tip = _xyz(landmarks, 8)
    middle_tip = _xyz(landmarks, 12)
    ring_tip = _xyz(landmarks, 16)
    pinky_tip = _xyz(landmarks, 20)

    index_open = _finger_extended(landmarks, 5, 6, 7, 8)
    middle_open = _finger_extended(landmarks, 9, 10, 11, 12)
    ring_open = _finger_extended(landmarks, 13, 14, 15, 16)
    pinky_open = _finger_extended(landmarks, 17, 18, 19, 20)
    thumb_open = _thumb_extended(landmarks)

    pinch_ratio = _distance(thumb_tip, index_tip) / palm_size
    victory_spread = _distance(index_tip, middle_tip) / palm_size
    thumb_up_vertical = (wrist[1] - thumb_tip[1]) / palm_size
    fist_compactness = max(
        _distance(index_tip, wrist),
        _distance(middle_tip, wrist),
        _distance(ring_tip, wrist),
        _distance(pinky_tip, wrist),
    ) / palm_size

    debug = (
        f"thumb={int(thumb_open)} index={int(index_open)} middle={int(middle_open)} "
        f"ring={int(ring_open)} pinky={int(pinky_open)} pinch={pinch_ratio:.2f}"
    )

    if pinch_ratio < 0.38 and middle_open and ring_open and pinky_open:
        confidence = max(0.0, min(0.99, 1.0 - pinch_ratio))
        return "ok", confidence, debug

    if thumb_open and index_open and middle_open and ring_open and pinky_open and pinch_ratio > 0.48:
        confidence = 0.88
        return "open_palm", confidence, debug

    if index_open and middle_open and not ring_open and not pinky_open and victory_spread > 0.32:
        confidence = max(0.0, min(0.95, 0.55 + victory_spread))
        return "victory", confidence, debug

    if index_open and not middle_open and not ring_open and not pinky_open:
        confidence = 0.82 if pinch_ratio > 0.38 else 0.70
        return "point", confidence, debug

    if thumb_open and not index_open and not middle_open and not ring_open and not pinky_open and thumb_up_vertical > 0.18:
        confidence = max(0.0, min(0.95, 0.60 + thumb_up_vertical))
        return "thumb_up", confidence, debug

    if not index_open and not middle_open and not ring_open and not pinky_open and fist_compactness < 1.55:
        confidence = max(0.0, min(0.92, 1.40 - fist_compactness))
        return "fist", confidence, debug

    return None, 0.0, debug


class CameraGestureEngine:
    def __init__(
        self,
        camera_index: int = 0,
        max_num_hands: int = 1,
        min_detection_confidence: float = 0.6,
        min_tracking_confidence: float = 0.6,
        smoothing_history: int = 7,
        smoothing_consensus: int = 4,
        mirror: bool = True,
    ) -> None:
        ensure_camera_dependencies()
        self._mirror = mirror
        self._mp_hands = mp.solutions.hands
        self._mp_drawing = mp.solutions.drawing_utils
        self._mp_styles = mp.solutions.drawing_styles
        self._hands = self._mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._capture = cv2.VideoCapture(camera_index)
        if not self._capture.isOpened():
            raise RuntimeError(f"Failed to open camera index {camera_index}.")
        self._smoother = TemporalLabelSmoother(history_len=smoothing_history, min_consensus=smoothing_consensus)
        self._last_stable_label: str | None = None

    @property
    def last_stable_label(self) -> str | None:
        return self._last_stable_label

    def read(self) -> tuple[Any, CameraGesturePrediction]:
        ok, frame = self._capture.read()
        if not ok:
            raise RuntimeError("Failed to read a frame from the camera.")
        if self._mirror:
            frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self._hands.process(rgb_frame)

        raw_label: str | None = None
        confidence = 0.0
        handedness: str | None = None
        debug_text = ""

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]
            if result.multi_handedness:
                handedness = result.multi_handedness[0].classification[0].label
            raw_label, confidence, debug_text = classify_finger_gesture(hand_landmarks, handedness)
            self._mp_drawing.draw_landmarks(
                frame,
                hand_landmarks,
                self._mp_hands.HAND_CONNECTIONS,
                self._mp_styles.get_default_hand_landmarks_style(),
                self._mp_styles.get_default_hand_connections_style(),
            )

        stable_label = self._smoother.push(raw_label if confidence >= 0.55 else None)
        self._last_stable_label = stable_label

        prediction = CameraGesturePrediction(
            raw_label=raw_label,
            stable_label=stable_label,
            confidence=confidence,
            handedness=handedness,
            debug_text=debug_text,
        )
        return frame, prediction

    def draw_overlay(self, frame: Any, prediction: CameraGesturePrediction, active_label: str | None) -> Any:
        overlay_lines = [
            f"camera_raw: {prediction.raw_label or '-'} ({prediction.confidence:.2f})",
            f"camera_stable: {prediction.stable_label or '-'}",
            f"active_label: {active_label or '-'}",
        ]
        if prediction.handedness:
            overlay_lines.append(f"handedness: {prediction.handedness}")
        if prediction.debug_text:
            overlay_lines.append(prediction.debug_text)

        y = 30
        for line in overlay_lines:
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 255, 30), 2, cv2.LINE_AA)
            y += 26
        return frame

    def close(self) -> None:
        self._capture.release()
        self._hands.close()
