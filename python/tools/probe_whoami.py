from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime

import serial

from ..common.dual_protocol import DEFAULT_BAUD


@dataclass
class ProbeSummary:
    found_addresses: list[str] = field(default_factory=list)
    whoami_by_address: dict[str, str] = field(default_factory=dict)
    found_count: str | None = None
    expected_68: str | None = None
    expected_69: str | None = None
    target_68: str | None = None
    target_69: str | None = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read the mpu_whoami_probe serial output and summarize it.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM16.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=8.0,
        help="How long to wait for a full probe block before timing out.",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print every raw serial line in addition to the parsed summary.",
    )
    return parser


def parse_probe_line(line: str, summary: ProbeSummary) -> None:
    if line.startswith("# i2c_found="):
        parts = line[2:].split()
        addr = parts[0].split("=", 1)[1]
        if addr not in summary.found_addresses:
            summary.found_addresses.append(addr)
        for token in parts[1:]:
            if token.startswith("whoami="):
                summary.whoami_by_address[addr] = token.split("=", 1)[1]
    elif line.startswith("# found_count="):
        summary.found_count = line.split("=", 1)[1]
    elif line.startswith("# expected_0x68="):
        summary.expected_68 = line.split("=", 1)[1]
    elif line.startswith("# expected_0x69="):
        summary.expected_69 = line.split("=", 1)[1]
    elif line.startswith("# probe_target=0x68 whoami="):
        summary.target_68 = line.rsplit("=", 1)[1]
    elif line.startswith("# probe_target=0x69 whoami="):
        summary.target_69 = line.rsplit("=", 1)[1]


def print_summary(summary: ProbeSummary) -> None:
    print("\nSummary")
    print(f"  found_count: {summary.found_count or '-'}")
    print(f"  expected 0x68: {summary.expected_68 or '-'}")
    print(f"  expected 0x69: {summary.expected_69 or '-'}")
    print(f"  target 0x68 WHO_AM_I: {summary.target_68 or '-'}")
    print(f"  target 0x69 WHO_AM_I: {summary.target_69 or '-'}")

    if summary.found_addresses:
        print("  detected devices:")
        for addr in summary.found_addresses:
            whoami = summary.whoami_by_address.get(addr, "-")
            print(f"    {addr} -> WHO_AM_I {whoami}")
    else:
        print("  detected devices: none")


def main() -> None:
    args = build_parser().parse_args()
    summary = ProbeSummary()
    deadline_note = f"{datetime.now().isoformat(timespec='seconds')}"

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print(f"[{deadline_note}] Listening on {args.port} @ {args.baud}")
        ser.reset_input_buffer()
        saw_probe_begin = False
        saw_probe_done = False
        deadline = datetime.now().timestamp() + args.timeout_seconds

        while datetime.now().timestamp() < deadline:
            raw_line = ser.readline().decode("utf-8", errors="ignore")
            if not raw_line:
                continue

            line = raw_line.strip()
            if not line:
                continue

            if args.raw:
                print(line)

            if line == "# probe_begin":
                saw_probe_begin = True
                summary = ProbeSummary()
                continue

            if not saw_probe_begin:
                continue

            parse_probe_line(line, summary)
            if line == "# probe_done":
                saw_probe_done = True
                break

        if not saw_probe_begin:
            raise TimeoutError(
                "Did not receive a probe block. Make sure the mpu_whoami_probe sketch is flashed and the port is correct."
            )
        if not saw_probe_done:
            raise TimeoutError(
                "Timed out before # probe_done. The board may have reset slowly or the serial output was interrupted."
            )

    print_summary(summary)


if __name__ == "__main__":
    main()
