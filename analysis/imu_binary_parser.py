from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import struct
from typing import BinaryIO, Iterator, Union

IMU_FILE_MAGIC = 0x31554D49
CURRENT_FORMAT_VERSION = 1

_HEADER_STRUCT = struct.Struct("<IIQIHH")
_BATCH_HEADER_STRUCT = struct.Struct("<QH")
_SAMPLE_STRUCT = struct.Struct("<hhhhhhh")

PathLike = Union[str, Path]


class BinaryParseError(Exception):
    """Raised when an IMU binary file cannot be parsed."""


@dataclass(frozen=True)
class FileHeader:
    magic: int
    version: int
    start_time_us: int
    sample_rate: int
    accel_fsr: int
    gyro_fsr: int


@dataclass(frozen=True)
class ImuSample:
    timestamp_us: int
    accel_x: int
    accel_y: int
    accel_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    temperature: int

    def accel_g(self, accel_fsr: int) -> tuple[float, float, float]:
        scale = accel_fsr / 32768.0
        return (
            self.accel_x * scale,
            self.accel_y * scale,
            self.accel_z * scale,
        )

    def gyro_dps(self, gyro_fsr: int) -> tuple[float, float, float]:
        scale = gyro_fsr / 32768.0
        return (
            self.gyro_x * scale,
            self.gyro_y * scale,
            self.gyro_z * scale,
        )


@dataclass(frozen=True)
class ParsedImuFile:
    header: FileHeader
    samples: list[ImuSample]


def _read_exact(stream: BinaryIO, size: int, what: str) -> bytes:
    data = stream.read(size)
    if len(data) != size:
        raise BinaryParseError(f"Unexpected EOF while reading {what}: expected {size} bytes, got {len(data)}")
    return data


def _parse_header(stream: BinaryIO) -> FileHeader:
    raw_header = _read_exact(stream, _HEADER_STRUCT.size, "file header")
    header = FileHeader(*_HEADER_STRUCT.unpack(raw_header))

    if header.magic != IMU_FILE_MAGIC:
        raise BinaryParseError(
            f"Invalid file magic: expected 0x{IMU_FILE_MAGIC:08X}, got 0x{header.magic:08X}"
        )

    if header.version != CURRENT_FORMAT_VERSION:
        raise BinaryParseError(
            f"Unsupported format version: expected {CURRENT_FORMAT_VERSION}, got {header.version}"
        )

    if header.sample_rate <= 0:
        raise BinaryParseError(f"Invalid sample rate in header: {header.sample_rate}")

    return header


def read_imu_header(file_path: PathLike) -> FileHeader:
    """Read and validate only the IMU file header."""
    path = Path(file_path)
    with path.open("rb") as stream:
        return _parse_header(stream)


def iter_imu_samples(file_path: PathLike) -> Iterator[ImuSample]:
    """Yield IMU samples from file in recording order."""
    path = Path(file_path)
    with path.open("rb") as stream:
        header = _parse_header(stream)
        sample_period_us = int(round(1_000_000 / header.sample_rate))

        while True:
            batch_header_raw = stream.read(_BATCH_HEADER_STRUCT.size)
            if not batch_header_raw:
                break

            if len(batch_header_raw) != _BATCH_HEADER_STRUCT.size:
                raise BinaryParseError(
                    "Unexpected EOF while reading batch header: "
                    f"expected {_BATCH_HEADER_STRUCT.size} bytes, got {len(batch_header_raw)}"
                )

            base_timestamp_us, sample_count = _BATCH_HEADER_STRUCT.unpack(batch_header_raw)
            batch_sample_bytes = sample_count * _SAMPLE_STRUCT.size
            sample_data = _read_exact(stream, batch_sample_bytes, f"batch samples ({sample_count})")

            for index in range(sample_count):
                offset = index * _SAMPLE_STRUCT.size
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature = _SAMPLE_STRUCT.unpack_from(
                    sample_data,
                    offset,
                )
                yield ImuSample(
                    timestamp_us=base_timestamp_us + index * sample_period_us,
                    accel_x=accel_x,
                    accel_y=accel_y,
                    accel_z=accel_z,
                    gyro_x=gyro_x,
                    gyro_y=gyro_y,
                    gyro_z=gyro_z,
                    temperature=temperature,
                )


def parse_imu_file(file_path: PathLike) -> ParsedImuFile:
    """Parse a complete IMU binary file into memory."""
    path = Path(file_path)
    header = read_imu_header(path)
    return ParsedImuFile(header=header, samples=list(iter_imu_samples(path)))
