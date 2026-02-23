#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a .mov video to downsampled .mp4 and name it after a matching .bin file."
        )
    )
    parser.add_argument("mov_file", type=Path, help="Path to source .mov file")
    parser.add_argument("bin_file", type=Path, help="Path to matching .bin file")
    return parser


def _validate_paths(mov_file: Path, bin_file: Path) -> tuple[Path, Path]:
    mov_file = mov_file.resolve()
    bin_file = bin_file.resolve()

    if not mov_file.exists() or not mov_file.is_file():
        raise FileNotFoundError(f"MOV file not found: {mov_file}")
    if not bin_file.exists() or not bin_file.is_file():
        raise FileNotFoundError(f"BIN file not found: {bin_file}")

    if mov_file.suffix.lower() != ".mov":
        raise ValueError(f"Expected .mov input file, got: {mov_file.name}")
    if bin_file.suffix.lower() != ".bin":
        raise ValueError(f"Expected .bin input file, got: {bin_file.name}")

    return mov_file, bin_file


def convert_and_name_video(mov_file: Path, bin_file: Path) -> Path:
    ffmpeg_path = shutil.which("ffmpeg")
    if ffmpeg_path is None:
        raise RuntimeError("ffmpeg was not found in PATH")

    mov_file, bin_file = _validate_paths(mov_file, bin_file)
    output_file = bin_file.with_suffix(".mp4")

    command = [
        ffmpeg_path,
        "-y",
        "-i",
        str(mov_file),
        "-vf",
        "scale=640:-2,fps=24",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "26",
        "-pix_fmt",
        "yuv420p",
        "-an",
        "-movflags",
        "+faststart",
        str(output_file),
    ]

    subprocess.run(command, check=True)

    mov_file.unlink()
    return output_file


def main() -> int:
    parser = _build_arg_parser()
    args = parser.parse_args()

    try:
        output_file = convert_and_name_video(args.mov_file, args.bin_file)
    except FileNotFoundError as error:
        print(error, file=sys.stderr)
        return 1
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    except subprocess.CalledProcessError as error:
        print(f"ffmpeg conversion failed with exit code {error.returncode}", file=sys.stderr)
        return 3
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 4

    print(f"Created: {output_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
