from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from sklearn.model_selection import StratifiedKFold, cross_val_predict

from features import extract_feature_vector, select_peak_window
from protocol import load_recording


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train a gesture classifier from labeled CSV trials.")
    parser.add_argument("--data-dir", type=Path, default=Path("data/raw"))
    parser.add_argument("--model-out", type=Path, default=Path("models/gesture_model.joblib"))
    parser.add_argument("--trees", type=int, default=300)
    return parser


def load_dataset(data_dir: Path) -> tuple[pd.DataFrame, np.ndarray]:
    feature_rows: list[dict[str, float]] = []
    labels: list[str] = []

    for path in sorted(data_dir.glob("*.csv")):
        recording = load_recording(path)
        if recording.data.empty:
            continue
        label = recording.metadata.get("label")
        if not label:
            raise ValueError(f"Missing label metadata in {path}")
        trimmed = select_peak_window(recording.data)
        feature_rows.append(extract_feature_vector(trimmed))
        labels.append(label)

    if not feature_rows:
        raise ValueError(f"No labeled recordings found in {data_dir}")

    return pd.DataFrame(feature_rows), np.array(labels)


def choose_splits(labels: np.ndarray) -> int:
    counts = Counter(labels)
    min_count = min(counts.values())
    if min_count < 2:
        return 0
    return min(5, min_count)


def main() -> None:
    args = build_parser().parse_args()
    features, labels = load_dataset(args.data_dir)

    model = RandomForestClassifier(
        n_estimators=args.trees,
        random_state=42,
        min_samples_leaf=2,
        class_weight="balanced_subsample",
    )
    model.fit(features, labels)

    report: dict[str, object] = {
        "sample_count": int(len(labels)),
        "labels": sorted(set(labels.tolist())),
        "class_distribution": dict(Counter(labels)),
        "feature_count": int(features.shape[1]),
    }

    cv_splits = choose_splits(labels)
    if cv_splits >= 2:
        splitter = StratifiedKFold(n_splits=cv_splits, shuffle=True, random_state=42)
        predicted = cross_val_predict(model, features, labels, cv=splitter)
        accuracy = accuracy_score(labels, predicted)
        confusion = confusion_matrix(labels, predicted, labels=report["labels"]).tolist()
        class_report = classification_report(labels, predicted, output_dict=True)
        report.update(
            {
                "cross_val_accuracy": accuracy,
                "confusion_matrix": confusion,
                "classification_report": class_report,
            }
        )
        print(f"Cross-val accuracy: {accuracy:.3f}")
        print(pd.DataFrame(confusion, index=report["labels"], columns=report["labels"]))
    else:
        print("Not enough samples per class for cross-validation. Collect at least 2 trials per gesture.")

    args.model_out.parent.mkdir(parents=True, exist_ok=True)
    bundle = {
        "model": model,
        "feature_names": list(features.columns),
        "labels": sorted(set(labels.tolist())),
        "report": report,
    }
    joblib.dump(bundle, args.model_out)
    report_path = args.model_out.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Saved model to {args.model_out}")
    print(f"Saved report to {report_path}")


if __name__ == "__main__":
    main()
