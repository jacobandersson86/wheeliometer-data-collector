#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path

try:
    from analysis import BinaryParseError, parse_imu_file
except ImportError:
    from imu_binary_parser import BinaryParseError, parse_imu_file


# MPU6886 datasheet (MPU-6886-000193+v1.1)
# TEMP_degC = TEMP_OUT / 326.8 + 25
# Temperature sensor operating range: -40°C to +85°C
MPU6886_TEMP_SENSITIVITY_LSB_PER_C = 326.8
MPU6886_TEMP_OFFSET_C = 25.0
MPU6886_TEMP_MIN_C = -40.0
MPU6886_TEMP_MAX_C = 85.0


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot wheeliometer IMU binary data")
    parser.add_argument("file", type=Path, help="Path to .bin file")
    return parser


def main() -> int:
    args = _build_arg_parser().parse_args()

    try:
        parsed = parse_imu_file(args.file)
    except FileNotFoundError:
        print(f"File not found: {args.file}", file=sys.stderr)
        return 1
    except BinaryParseError as error:
        print(f"Parse error: {error}", file=sys.stderr)
        return 2

    if not parsed.samples:
        print("No samples found in file", file=sys.stderr)
        return 3

    try:
        plt = importlib.import_module("matplotlib.pyplot")
    except ImportError:
        print("Missing dependency: matplotlib. Install with 'pip install matplotlib'", file=sys.stderr)
        return 4

    header = parsed.header
    first_ts = parsed.samples[0].timestamp_us

    times_s = [(sample.timestamp_us - first_ts) / 1_000_000.0 for sample in parsed.samples]

    accel_scale = header.accel_fsr / 32768.0
    gyro_scale = header.gyro_fsr / 32768.0

    accel_x_g = [sample.accel_x * accel_scale for sample in parsed.samples]
    accel_y_g = [sample.accel_y * accel_scale for sample in parsed.samples]
    accel_z_g = [sample.accel_z * accel_scale for sample in parsed.samples]

    gyro_x_dps = [sample.gyro_x * gyro_scale for sample in parsed.samples]
    gyro_y_dps = [sample.gyro_y * gyro_scale for sample in parsed.samples]
    gyro_z_dps = [sample.gyro_z * gyro_scale for sample in parsed.samples]

    temperature_c = [
        (sample.temperature / MPU6886_TEMP_SENSITIVITY_LSB_PER_C) + MPU6886_TEMP_OFFSET_C
        for sample in parsed.samples
    ]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))

    accel_axis = axes[0]
    accel_axis.plot(times_s, accel_x_g, label="accel_x")
    accel_axis.plot(times_s, accel_y_g, label="accel_y")
    accel_axis.plot(times_s, accel_z_g, label="accel_z")
    accel_axis.set_ylabel("Acceleration [g]")
    accel_axis.set_ylim(-header.accel_fsr, header.accel_fsr)
    accel_axis.grid(True)
    accel_axis.legend(loc="upper right")

    gyro_axis = axes[1]
    gyro_axis.plot(times_s, gyro_x_dps, label="gyro_x")
    gyro_axis.plot(times_s, gyro_y_dps, label="gyro_y")
    gyro_axis.plot(times_s, gyro_z_dps, label="gyro_z")
    gyro_axis.set_ylabel("Gyro [dps]")
    gyro_axis.set_ylim(-header.gyro_fsr, header.gyro_fsr)
    gyro_axis.grid(True)
    gyro_axis.legend(loc="upper right")

    temp_axis = axes[2]
    temp_axis.plot(times_s, temperature_c, label="temperature")
    temp_axis.set_ylabel("Temperature [°C]")
    temp_axis.set_ylim(MPU6886_TEMP_MIN_C, MPU6886_TEMP_MAX_C)
    temp_axis.set_xlabel("Time [s]")
    temp_axis.grid(True)
    temp_axis.legend(loc="upper right")

    fig.suptitle(args.file.name)
    fig.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
