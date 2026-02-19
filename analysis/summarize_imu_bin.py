#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, Tuple

try:
    from analysis import BinaryParseError, iter_imu_samples, read_imu_header
except ImportError:
    from imu_binary_parser import BinaryParseError, iter_imu_samples, read_imu_header


FieldRange = Dict[str, Tuple[int, int]]


def _update_range(ranges: FieldRange, name: str, value: int) -> None:
    if name not in ranges:
        ranges[name] = (value, value)
        return

    current_min, current_max = ranges[name]
    ranges[name] = (min(current_min, value), max(current_max, value))


def summarize_imu_file(file_path: Path) -> int:
    header = read_imu_header(file_path)

    sample_count = 0
    first_timestamp_us = None
    last_timestamp_us = None
    ranges: FieldRange = {}

    for sample in iter_imu_samples(file_path):
        if first_timestamp_us is None:
            first_timestamp_us = sample.timestamp_us
        last_timestamp_us = sample.timestamp_us
        sample_count += 1

        _update_range(ranges, "accel_x", sample.accel_x)
        _update_range(ranges, "accel_y", sample.accel_y)
        _update_range(ranges, "accel_z", sample.accel_z)
        _update_range(ranges, "gyro_x", sample.gyro_x)
        _update_range(ranges, "gyro_y", sample.gyro_y)
        _update_range(ranges, "gyro_z", sample.gyro_z)
        _update_range(ranges, "temperature", sample.temperature)

    duration_us = 0
    if first_timestamp_us is not None and last_timestamp_us is not None:
        duration_us = max(0, last_timestamp_us - first_timestamp_us)

    expected_duration_s = 0.0
    if sample_count > 1 and header.sample_rate > 0:
        expected_duration_s = (sample_count - 1) / header.sample_rate

    measured_rate_hz = 0.0
    if duration_us > 0 and sample_count > 1:
        measured_rate_hz = (sample_count - 1) / (duration_us / 1_000_000.0)

    print(f"File: {file_path}")
    print(f"Version: {header.version}")
    print(f"Sample rate (header): {header.sample_rate} Hz")
    print(f"Samples: {sample_count}")
    print(f"Duration: {duration_us / 1_000_000.0:.3f} s")
    print(f"Expected duration from header rate: {expected_duration_s:.3f} s")
    if measured_rate_hz > 0.0:
        print(f"Measured sample rate: {measured_rate_hz:.2f} Hz")
    print(f"Accel FSR: ±{header.accel_fsr} g")
    print(f"Gyro FSR: ±{header.gyro_fsr} dps")

    if sample_count == 0:
        print("No samples found in file")
        return 0

    print("Ranges (raw int16):")
    for field in ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z", "temperature"]:
        min_value, max_value = ranges[field]
        print(f"  {field:12s} min={min_value:6d} max={max_value:6d}")

    return 0


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Summarize a wheeliometer IMU binary file")
    parser.add_argument("file", type=Path, help="Path to .bin file")
    return parser


def main() -> int:
    parser = _build_arg_parser()
    args = parser.parse_args()

    try:
        return summarize_imu_file(args.file)
    except FileNotFoundError:
        print(f"File not found: {args.file}", file=sys.stderr)
        return 1
    except BinaryParseError as error:
        print(f"Parse error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
