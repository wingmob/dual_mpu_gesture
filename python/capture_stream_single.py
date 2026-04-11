from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import serial

from protocol_single import DEFAULT_BAUD


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stream single MPU6050 CSV frames to screen or file.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--out", type=Path, help="Optional path to save the raw stream.")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output_handle = None

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        output_handle = args.out.open("w", encoding="utf-8", newline="")
        print(f"Saving stream to {args.out}")

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print(f"[{datetime.now().isoformat(timespec='seconds')}] Listening on {args.port} @ {args.baud}")
        try:
            while True:
                raw_line = ser.readline().decode("utf-8", errors="ignore")
                if not raw_line:
                    continue
                print(raw_line.rstrip())
                if output_handle:
                    output_handle.write(raw_line)
        except KeyboardInterrupt:
            print("Stopped.")
        finally:
            if output_handle:
                output_handle.close()


if __name__ == "__main__":
    main()
