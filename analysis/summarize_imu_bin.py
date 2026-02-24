#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, Tuple

try:
    from analysis import BinaryParseError, iter_imu_samples, read_imu_header
except ImportError:
    from imu_binary_parser import BinaryParseError, iter_imu_samples, read_imu_header


FieldRange = Dict[str, Tuple[int, int]]

_HEADER_STRUCT = struct.Struct("<IIQIHH")
_BATCH_HEADER_STRUCT = struct.Struct("<QH")
_SAMPLE_SIZE_BYTES = 14


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

    period_us = int(round(1_000_000 / header.sample_rate)) if header.sample_rate > 0 else 0
    batch_count = 0
    batch_size_histogram: Counter[int] = Counter()
    batch_gap_us_values: list[int] = []
    estimated_missing_samples = 0

    with file_path.open("rb") as stream:
        stream.seek(_HEADER_STRUCT.size)

        previous_batch_base_us = None
        previous_batch_count = None

        while True:
            batch_header_raw = stream.read(_BATCH_HEADER_STRUCT.size)
            if not batch_header_raw:
                break

            if len(batch_header_raw) != _BATCH_HEADER_STRUCT.size:
                raise BinaryParseError(
                    "Unexpected EOF while reading batch header: "
                    f"expected {_BATCH_HEADER_STRUCT.size} bytes, got {len(batch_header_raw)}"
                )

            batch_base_timestamp_us, batch_sample_count = _BATCH_HEADER_STRUCT.unpack(batch_header_raw)
            sample_payload_size = batch_sample_count * _SAMPLE_SIZE_BYTES
            payload = stream.read(sample_payload_size)
            if len(payload) != sample_payload_size:
                raise BinaryParseError(
                    "Unexpected EOF while reading batch payload: "
                    f"expected {sample_payload_size} bytes, got {len(payload)}"
                )

            batch_count += 1
            batch_size_histogram[batch_sample_count] += 1

            if (
                period_us > 0
                and previous_batch_base_us is not None
                and previous_batch_count is not None
            ):
                expected_next_batch_base_us = previous_batch_base_us + previous_batch_count * period_us
                batch_gap_us = int(batch_base_timestamp_us - expected_next_batch_base_us)
                batch_gap_us_values.append(batch_gap_us)
                if batch_gap_us > 0:
                    estimated_missing_samples += int(round(batch_gap_us / period_us))

            previous_batch_base_us = batch_base_timestamp_us
            previous_batch_count = batch_sample_count

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

    if batch_count > 0:
        print(f"Batches: {batch_count}")
        most_common_batch_sizes = batch_size_histogram.most_common(3)
        formatted_sizes = ", ".join([f"{size} ({count})" for size, count in most_common_batch_sizes])
        print(f"Most common batch sizes: {formatted_sizes}")

    if batch_gap_us_values:
        gap_min_us = min(batch_gap_us_values)
        gap_max_us = max(batch_gap_us_values)
        gap_avg_us = sum(batch_gap_us_values) / len(batch_gap_us_values)
        gap_over_2_periods = sum(1 for gap in batch_gap_us_values if abs(gap) > 2 * period_us)
        gap_over_10_periods = sum(1 for gap in batch_gap_us_values if abs(gap) > 10 * period_us)
        print(
            "Inter-batch timing gap (actual next base - expected next base): "
            f"min={gap_min_us} us avg={gap_avg_us:.1f} us max={gap_max_us} us"
        )
        print(
            f"Large inter-batch gaps: >2 periods={gap_over_2_periods}/{len(batch_gap_us_values)}, "
            f">10 periods={gap_over_10_periods}/{len(batch_gap_us_values)}"
        )
        print(f"Estimated missing samples from positive gaps: {estimated_missing_samples}")

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
