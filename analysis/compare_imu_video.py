#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    from analysis import BinaryParseError, parse_imu_file
    from analysis.orientation_fusion import (
        fuse_pose,
        parse_sensor_to_body_matrix,
        pose_matrix_4x4,
        quat_to_rotation_matrix,
        rotate_vector,
        select_frame_indices,
        validate_sensor_to_body_matrix,
    )
except ImportError:
    from imu_binary_parser import BinaryParseError, parse_imu_file
    from orientation_fusion import (
        fuse_pose,
        parse_sensor_to_body_matrix,
        pose_matrix_4x4,
        quat_to_rotation_matrix,
        rotate_vector,
        select_frame_indices,
        validate_sensor_to_body_matrix,
    )


@dataclass(frozen=True)
class TriangleMesh:
    triangles: list[list[tuple[float, float, float]]]


def _default_model_dir() -> Path:
    return Path(__file__).resolve().parent / "3d_models" / "proprietary"


def _find_default_stl() -> Path | None:
    model_dir = _default_model_dir()
    if not model_dir.exists():
        return None

    stl_files = sorted(model_dir.glob("*.stl"))
    if not stl_files:
        return None
    return stl_files[0]


def _read_binary_stl(data: bytes) -> list[list[tuple[float, float, float]]]:
    if len(data) < 84:
        raise ValueError("Binary STL too small")

    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) < expected_size:
        raise ValueError("Binary STL truncated")

    triangles: list[list[tuple[float, float, float]]] = []
    offset = 84
    for _ in range(triangle_count):
        offset += 12
        v1 = struct.unpack_from("<fff", data, offset)
        offset += 12
        v2 = struct.unpack_from("<fff", data, offset)
        offset += 12
        v3 = struct.unpack_from("<fff", data, offset)
        offset += 12
        offset += 2
        triangles.append([v1, v2, v3])

    return triangles


def _read_ascii_stl(text: str) -> list[list[tuple[float, float, float]]]:
    triangles: list[list[tuple[float, float, float]]] = []
    current_vertices: list[tuple[float, float, float]] = []

    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.lower().startswith("vertex "):
            continue
        parts = stripped.split()
        if len(parts) != 4:
            continue
        vertex = (float(parts[1]), float(parts[2]), float(parts[3]))
        current_vertices.append(vertex)
        if len(current_vertices) == 3:
            triangles.append(current_vertices)
            current_vertices = []

    if not triangles:
        raise ValueError("No triangles found in ASCII STL")

    return triangles


def _load_stl_mesh(path: Path) -> TriangleMesh:
    data = path.read_bytes()
    if len(data) < 6:
        raise ValueError("STL file is empty or invalid")

    is_ascii_candidate = data[:5].lower() == b"solid"

    if is_ascii_candidate:
        try:
            text = data.decode("utf-8")
            return TriangleMesh(_read_ascii_stl(text))
        except Exception:
            pass

    return TriangleMesh(_read_binary_stl(data))


def _mesh_bounds(mesh: TriangleMesh) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    xs = [v[0] for tri in mesh.triangles for v in tri]
    ys = [v[1] for tri in mesh.triangles for v in tri]
    zs = [v[2] for tri in mesh.triangles for v in tri]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def _mesh_spans(mesh: TriangleMesh) -> tuple[float, float, float]:
    minimum, maximum = _mesh_bounds(mesh)
    return (
        maximum[0] - minimum[0],
        maximum[1] - minimum[1],
        maximum[2] - minimum[2],
    )


def _center_and_scale_mesh(mesh: TriangleMesh, scale: float) -> TriangleMesh:
    minimum, maximum = _mesh_bounds(mesh)
    center = (
        (minimum[0] + maximum[0]) * 0.5,
        (minimum[1] + maximum[1]) * 0.5,
        (minimum[2] + maximum[2]) * 0.5,
    )

    normalized = []
    for tri in mesh.triangles:
        normalized_tri = []
        for vertex in tri:
            normalized_tri.append(
                (
                    (vertex[0] - center[0]) * scale,
                    (vertex[1] - center[1]) * scale,
                    (vertex[2] - center[2]) * scale,
                )
            )
        normalized.append(normalized_tri)

    return TriangleMesh(normalized)


def _normalize_mesh(mesh: TriangleMesh, target_size: float) -> TriangleMesh:
    spans = _mesh_spans(mesh)
    max_span = max(spans)
    scale = 1.0 if max_span <= 0.0 else target_size / max_span
    return _center_and_scale_mesh(mesh, scale)


def _parse_model_size_mm(raw_value: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in raw_value.split(",") if part.strip()]
    if len(parts) != 3:
        raise ValueError("--model-size-mm must have 3 comma-separated values: length,width,height")

    values = tuple(float(part) for part in parts)
    if any(value <= 0.0 for value in values):
        raise ValueError("--model-size-mm values must be > 0")
    return values


def _fit_mesh_to_size_mm(mesh: TriangleMesh, size_mm: tuple[float, float, float]) -> tuple[TriangleMesh, tuple[float, float, float]]:
    raw_spans = _mesh_spans(mesh)
    target_spans_m = tuple(value / 1000.0 for value in size_mm)

    raw_sorted = sorted(raw_spans, reverse=True)
    target_sorted = sorted(target_spans_m, reverse=True)
    ratios = [target / raw for raw, target in zip(raw_sorted, target_sorted) if raw > 0.0]
    scale = 1.0 if not ratios else sum(ratios) / len(ratios)

    fitted = _center_and_scale_mesh(mesh, scale)
    fitted_spans = _mesh_spans(fitted)
    return fitted, fitted_spans


def _stl_unit_scale_to_meters(stl_unit: str) -> float:
    if stl_unit == "mm":
        return 0.001
    if stl_unit == "m":
        return 1.0
    raise ValueError(f"Unsupported STL unit: {stl_unit}")


def _build_cube_vertices(size: float) -> list[tuple[float, float, float]]:
    s = size / 2.0
    return [
        (-s, -s, -s),
        (s, -s, -s),
        (s, s, -s),
        (-s, s, -s),
        (-s, -s, s),
        (s, -s, s),
        (s, s, s),
        (-s, s, s),
    ]


def _build_cube_triangles(size: float) -> TriangleMesh:
    vertices = _build_cube_vertices(size)
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]
    triangles = [[vertices[a], vertices[b], vertices[c]] for a, b, c in faces]
    return TriangleMesh(triangles)


def _mesh_to_arrays(mesh: TriangleMesh) -> tuple[np.ndarray, np.ndarray]:
    triangle_count = len(mesh.triangles)
    vertexes = np.zeros((triangle_count * 3, 3), dtype=np.float32)
    faces = np.zeros((triangle_count, 3), dtype=np.uint32)

    for triangle_index, tri in enumerate(mesh.triangles):
        base = triangle_index * 3
        vertexes[base] = tri[0]
        vertexes[base + 1] = tri[1]
        vertexes[base + 2] = tri[2]
        faces[triangle_index] = [base, base + 1, base + 2]

    return vertexes, faces


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay IMU pose beside a video")
    parser.add_argument("file", type=Path, help="Path to .bin file")
    parser.add_argument("video", type=Path, help="Path to video file")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier (default: 1.0)")
    parser.add_argument("--render-fps", type=float, default=60.0, help="Animation frame rate (default: 60)")
    parser.add_argument("--cube-size", type=float, default=1.0, help="Cube side length (default: 1.0)")
    parser.add_argument(
        "--model",
        type=Path,
        default=None,
        help="Path to STL model. Defaults to first *.stl in analysis/3d_models/proprietary",
    )
    parser.add_argument(
        "--model-size-mm",
        type=str,
        default=None,
        help=(
            "Expected physical STL dimensions in mm: length,width,height (e.g. 48,24,13.5). "
            "When set, mesh is uniformly scaled to real size in meters instead of cube-size normalization."
        ),
    )
    parser.add_argument(
        "--stl-unit",
        type=str,
        choices=("mm", "m"),
        default="mm",
        help="Unit used by STL coordinates when --model-size-mm is not provided (default: mm)",
    )
    parser.add_argument(
        "--use-cube",
        action="store_true",
        help="Force cube rendering (skip STL load) for faster playback",
    )
    parser.add_argument("--kp", type=float, default=1.8, help="Accelerometer correction gain (default: 1.8)")
    parser.add_argument(
        "--video-offset-s",
        type=float,
        default=0.0,
        help="Initial video offset in seconds. Positive means video is delayed vs IMU timeline.",
    )
    parser.add_argument(
        "--sensor-to-body",
        type=str,
        default=None,
        help=(
            "3x3 sensor->body rotation matrix in row-major CSV: "
            "r11,r12,r13,r21,r22,r23,r31,r32,r33 "
            "(default is -1,0,0,0,-1,0,0,0,1)"
        ),
    )
    parser.add_argument(
        "--translation-scale",
        type=float,
        default=1.0,
        help="Scale factor for estimated translation (default: 1.0)",
    )
    parser.add_argument(
        "--rotation-only",
        action="store_true",
        help="Render rotation only (translation is still computed internally)",
    )
    return parser


def main() -> int:
    args = _build_arg_parser().parse_args()

    if args.speed <= 0.0:
        print("--speed must be > 0", file=sys.stderr)
        return 2
    if args.render_fps <= 0.0:
        print("--render-fps must be > 0", file=sys.stderr)
        return 2
    if args.cube_size <= 0.0:
        print("--cube-size must be > 0", file=sys.stderr)
        return 2
    if args.translation_scale <= 0.0:
        print("--translation-scale must be > 0", file=sys.stderr)
        return 2

    model_size_mm: tuple[float, float, float] | None = None
    if args.model_size_mm is not None:
        try:
            model_size_mm = _parse_model_size_mm(args.model_size_mm)
        except ValueError as error:
            print(f"Invalid --model-size-mm: {error}", file=sys.stderr)
            return 2

    try:
        sensor_to_body_matrix = parse_sensor_to_body_matrix(args.sensor_to_body)
    except ValueError as error:
        print(f"Invalid --sensor-to-body: {error}", file=sys.stderr)
        return 2

    matrix_ok, matrix_message = validate_sensor_to_body_matrix(sensor_to_body_matrix)
    if not matrix_ok:
        print(f"WARNING: {matrix_message}", file=sys.stderr)

    try:
        parsed = parse_imu_file(args.file)
    except FileNotFoundError:
        print(f"File not found: {args.file}", file=sys.stderr)
        return 1
    except BinaryParseError as error:
        print(f"Parse error: {error}", file=sys.stderr)
        return 1

    if len(parsed.samples) < 2:
        print("Need at least 2 samples to replay pose", file=sys.stderr)
        return 3

    model_path = args.model if args.model is not None else _find_default_stl()

    pose_samples = fuse_pose(
        parsed.samples,
        accel_fsr=parsed.header.accel_fsr,
        gyro_fsr=parsed.header.gyro_fsr,
        kp=args.kp,
        sensor_to_body_matrix=sensor_to_body_matrix,
    )

    try:
        qt_module = importlib.import_module("pyqtgraph.Qt")
        QtCore = qt_module.QtCore
        QtGui = qt_module.QtGui
        QtWidgets = qt_module.QtWidgets
        gl = importlib.import_module("pyqtgraph.opengl")
        cv2 = importlib.import_module("cv2")
    except ImportError as error:
        print(
            "Missing dependency. Install with "
            "'pip install pyqtgraph PyQt6 numpy PyOpenGL opencv-python'",
            file=sys.stderr,
        )
        print(f"Details: {error}", file=sys.stderr)
        print(f"Python executable: {sys.executable}", file=sys.stderr)
        return 4

    video_capture = cv2.VideoCapture(str(args.video))
    if not video_capture.isOpened():
        print(f"Failed to open video: {args.video}", file=sys.stderr)
        return 5

    video_fps = float(video_capture.get(cv2.CAP_PROP_FPS))
    if video_fps <= 0.0:
        video_fps = 30.0

    video_frame_count = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
    video_duration_s = video_frame_count / video_fps if video_frame_count > 0 else 0.0

    mesh: TriangleMesh
    visual_size = args.cube_size
    if args.use_cube:
        print("Using cube model (--use-cube)")
        mesh = _build_cube_triangles(args.cube_size)
    elif model_path is not None:
        try:
            raw_mesh = _load_stl_mesh(model_path)
            raw_spans = _mesh_spans(raw_mesh)
            print(
                "Raw STL bounds (model units): "
                f"{raw_spans[0]:.6f} x {raw_spans[1]:.6f} x {raw_spans[2]:.6f}"
            )

            if model_size_mm is None:
                unit_scale = _stl_unit_scale_to_meters(args.stl_unit)
                mesh = _center_and_scale_mesh(raw_mesh, unit_scale)
                unit_spans = _mesh_spans(mesh)
                print(
                    f"Interpreted STL using --stl-unit={args.stl_unit}; bounds (meters): "
                    f"{unit_spans[0]:.6f} x {unit_spans[1]:.6f} x {unit_spans[2]:.6f}"
                )
            else:
                mesh, fitted_spans = _fit_mesh_to_size_mm(raw_mesh, model_size_mm)
                target_m = tuple(value / 1000.0 for value in model_size_mm)
                print(
                    "Scaled STL bounds (meters): "
                    f"{fitted_spans[0]:.6f} x {fitted_spans[1]:.6f} x {fitted_spans[2]:.6f} "
                    f"(target ≈ {target_m[0]:.6f} x {target_m[1]:.6f} x {target_m[2]:.6f})"
                )

            print(f"Using STL model: {model_path}")
        except Exception as error:
            print(f"Failed to load STL model ({model_path}): {error}", file=sys.stderr)
            print("Falling back to cube model", file=sys.stderr)
            mesh = _build_cube_triangles(args.cube_size)
    else:
        mesh = _build_cube_triangles(args.cube_size)

    spans = _mesh_spans(mesh)
    visual_size = max(spans)
    if visual_size <= 0.0:
        visual_size = args.cube_size

    vertices, faces = _mesh_to_arrays(mesh)
    frame_indices = select_frame_indices(pose_samples, args.render_fps)
    if not frame_indices:
        print("No frames available for replay", file=sys.stderr)
        return 3

    imu_duration_s = (pose_samples[-1].timestamp_us - pose_samples[0].timestamp_us) / 1_000_000.0
    total_duration_s = max(imu_duration_s, video_duration_s)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    main_window = QtWidgets.QWidget()
    main_window.setWindowTitle(f"IMU + Video Compare - {args.file.name}")
    main_layout = QtWidgets.QVBoxLayout(main_window)
    main_layout.setContentsMargins(8, 8, 8, 8)
    main_layout.setSpacing(8)

    content_layout = QtWidgets.QHBoxLayout()
    content_layout.setContentsMargins(0, 0, 0, 0)
    content_layout.setSpacing(8)

    view = gl.GLViewWidget()
    view.setCameraPosition(distance=max(visual_size * 4.0, 0.2), elevation=20, azimuth=45)
    view.setBackgroundColor((48, 48, 48))
    content_layout.addWidget(view, stretch=1)

    video_label = QtWidgets.QLabel()
    video_label.setMinimumSize(640, 360)
    video_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    video_label.setStyleSheet("background-color: #111; color: #ddd;")
    video_label.setText("Video")
    content_layout.addWidget(video_label, stretch=1)

    main_layout.addLayout(content_layout, stretch=1)

    grid = gl.GLGridItem()
    grid.setSize(visual_size * 4, visual_size * 4)
    grid.setSpacing(visual_size / 2, visual_size / 2)
    view.addItem(grid)

    axis_item = gl.GLAxisItem()
    axis_item.setSize(visual_size * 1.5, visual_size * 1.5, visual_size * 1.5)
    view.addItem(axis_item)

    mesh_item = gl.GLMeshItem(
        vertexes=vertices,
        faces=faces,
        smooth=False,
        drawEdges=True,
        drawFaces=True,
        edgeColor=(0.2, 0.2, 0.2, 0.9),
        color=(1.0, 0.55, 0.1, 0.9),
        shader="balloon",
    )
    view.addItem(mesh_item)

    body_axis_line = gl.GLLinePlotItem(
        pos=np.array([[0.0, 0.0, 0.0], [0.0, 0.0, visual_size * 0.8]], dtype=np.float32),
        color=(1.0, 0.1, 0.1, 1.0),
        width=2.0,
        antialias=True,
    )
    view.addItem(body_axis_line)

    controls_layout = QtWidgets.QHBoxLayout()
    controls_layout.setContentsMargins(0, 0, 0, 0)
    controls_layout.setSpacing(8)

    play_pause_button = QtWidgets.QPushButton("Pause")
    controls_layout.addWidget(play_pause_button)

    timeline_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
    timeline_slider.setMinimum(0)
    slider_max = max(1, int(total_duration_s * args.render_fps))
    timeline_slider.setMaximum(slider_max)
    timeline_slider.setSingleStep(1)
    timeline_slider.setPageStep(max(1, int(args.render_fps // 2)))
    controls_layout.addWidget(timeline_slider, stretch=1)

    time_label = QtWidgets.QLabel("0.00s")
    controls_layout.addWidget(time_label)

    offset_label = QtWidgets.QLabel(f"Video offset: {args.video_offset_s:+.3f}s")
    controls_layout.addWidget(offset_label)

    main_layout.addLayout(controls_layout)

    main_window.resize(1600, 900)
    main_window.show()

    start_ts = pose_samples[0].timestamp_us
    frame_state = {
        "index": 0,
        "playing": True,
        "dragging": False,
        "resume_after_drag": False,
        "suppress_slider_signal": False,
    }
    sync_state = {
        "video_offset_s": args.video_offset_s,
    }

    video_state = {
        "current_frame": -1,
        "current_pixmap": None,
    }

    def _video_frame_for_time(time_s: float) -> int:
        if video_frame_count <= 0:
            return 0
        frame = int(round(time_s * video_fps))
        if frame < 0:
            frame = 0
        if frame >= video_frame_count:
            frame = video_frame_count - 1
        return frame

    def _render_video_frame(frame_rgb: np.ndarray) -> None:
        h, w, channels = frame_rgb.shape
        bytes_per_line = channels * w
        image = QtGui.QImage(frame_rgb.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(image)
        video_state["current_pixmap"] = pixmap
        scaled = pixmap.scaled(
            video_label.size(),
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation,
        )
        video_label.setPixmap(scaled)

    def _set_video_frame(time_s: float) -> None:
        if video_frame_count <= 0:
            return

        frame_index = _video_frame_for_time(time_s)

        if frame_index == video_state["current_frame"] and video_state["current_pixmap"] is not None:
            scaled = video_state["current_pixmap"].scaled(
                video_label.size(),
                QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                QtCore.Qt.TransformationMode.SmoothTransformation,
            )
            video_label.setPixmap(scaled)
            return

        seek_needed = True
        if video_state["current_frame"] >= 0:
            if frame_index == video_state["current_frame"] + 1:
                seek_needed = False
            elif frame_index > video_state["current_frame"] and frame_index - video_state["current_frame"] <= 3:
                seek_needed = False

        ok = False
        frame_bgr = None

        if seek_needed:
            video_capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, frame_bgr = video_capture.read()
            if ok:
                video_state["current_frame"] = frame_index
        else:
            while video_state["current_frame"] < frame_index:
                ok, frame_bgr = video_capture.read()
                if not ok:
                    break
                video_state["current_frame"] += 1

        if not ok:
            return

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        _render_video_frame(frame_rgb)

    def _find_imu_frame_for_time(time_s: float) -> int:
        target_ts = start_ts + int(time_s * 1_000_000.0)
        index = frame_state["index"]
        if index < 0:
            index = 0
        if index >= len(frame_indices):
            index = len(frame_indices) - 1

        while index + 1 < len(frame_indices) and pose_samples[frame_indices[index + 1]].timestamp_us <= target_ts:
            index += 1
        while index > 0 and pose_samples[frame_indices[index]].timestamp_us > target_ts:
            index -= 1
        return index

    def apply_frame(frame_number: int, *, timeline_time_s: float | None = None) -> None:
        sample_index = frame_indices[frame_number]
        sample = pose_samples[sample_index]
        q = sample.quaternion
        rotation3 = quat_to_rotation_matrix(q)
        if args.rotation_only:
            translation = (0.0, 0.0, 0.0)
        else:
            translation = (
                sample.position_m[0] * args.translation_scale,
                sample.position_m[1] * args.translation_scale,
                sample.position_m[2] * args.translation_scale,
            )
        pose4 = pose_matrix_4x4(rotation3, translation)

        transform = QtGui.QMatrix4x4(*pose4.reshape(-1).tolist())
        mesh_item.resetTransform()
        mesh_item.setTransform(transform)

        body_z = rotate_vector(rotation3, (0.0, 0.0, visual_size * 0.8))
        body_axis_line.setData(
            pos=np.array(
                [
                    [translation[0], translation[1], translation[2]],
                    [translation[0] + body_z[0], translation[1] + body_z[1], translation[2] + body_z[2]],
                ],
                dtype=np.float32,
            ),
            color=(1.0, 0.1, 0.1, 1.0),
            width=2.0,
            antialias=True,
        )

        imu_time_s = (sample.timestamp_us - start_ts) / 1_000_000.0
        display_time_s = imu_time_s if timeline_time_s is None else timeline_time_s
        video_time_s = display_time_s + sync_state["video_offset_s"]
        if video_time_s < 0.0:
            video_time_s = 0.0

        _set_video_frame(video_time_s)
        main_window.setWindowTitle(
            f"IMU + Video Compare - {args.file.name} | t={display_time_s:.2f}s | video={video_time_s:.2f}s"
        )
        time_label.setText(f"{display_time_s:.2f}s")

    def set_video_offset(new_offset_s: float) -> None:
        sync_state["video_offset_s"] = new_offset_s
        offset_label.setText(f"Video offset: {new_offset_s:+.3f}s")
        _apply_slider_value(timeline_slider.value())

    def nudge_video_offset(delta_s: float) -> None:
        set_video_offset(sync_state["video_offset_s"] + delta_s)

    def set_frame(frame_number: int, *, update_slider: bool, timeline_time_s: float | None = None) -> None:
        if frame_number < 0:
            frame_number = 0
        if frame_number >= len(frame_indices):
            frame_number = len(frame_indices) - 1

        frame_state["index"] = frame_number
        apply_frame(frame_number, timeline_time_s=timeline_time_s)

        if update_slider and timeline_time_s is not None:
            slider_value = int(round(timeline_time_s * args.render_fps))
            slider_value = max(0, min(timeline_slider.maximum(), slider_value))
            frame_state["suppress_slider_signal"] = True
            timeline_slider.setValue(slider_value)
            frame_state["suppress_slider_signal"] = False

    def set_playing(playing: bool) -> None:
        frame_state["playing"] = playing
        play_pause_button.setText("Pause" if playing else "Play")

    interval_ms = 1000.0 / (args.render_fps * args.speed)
    if interval_ms < 1.0:
        interval_ms = 1.0
    interval_ms_int = int(round(interval_ms))

    def on_tick() -> None:
        if not frame_state["playing"] or frame_state["dragging"]:
            return

        current_slider_time = timeline_slider.value() / args.render_fps
        next_time = current_slider_time + (1.0 / args.render_fps)
        if next_time >= total_duration_s:
            next_time = total_duration_s
            set_playing(False)

        imu_frame = _find_imu_frame_for_time(next_time)
        set_frame(imu_frame, update_slider=True, timeline_time_s=next_time)

    def on_play_pause_clicked() -> None:
        if frame_state["playing"]:
            set_playing(False)
            return

        if timeline_slider.value() >= timeline_slider.maximum():
            set_frame(0, update_slider=True, timeline_time_s=0.0)

        set_playing(True)

    def on_slider_pressed() -> None:
        frame_state["dragging"] = True
        frame_state["resume_after_drag"] = frame_state["playing"]
        set_playing(False)

    def _apply_slider_value(slider_value: int) -> None:
        timeline_time_s = slider_value / args.render_fps
        imu_frame = _find_imu_frame_for_time(timeline_time_s)
        set_frame(imu_frame, update_slider=False, timeline_time_s=timeline_time_s)

    def on_slider_moved(slider_value: int) -> None:
        _apply_slider_value(slider_value)

    def on_slider_released() -> None:
        frame_state["dragging"] = False
        if frame_state["resume_after_drag"]:
            set_playing(True)

    def on_slider_changed(slider_value: int) -> None:
        if frame_state["suppress_slider_signal"]:
            return
        if not frame_state["dragging"]:
            _apply_slider_value(slider_value)

    def seek_by_frames(delta: int) -> None:
        if delta == 0:
            return
        target = timeline_slider.value() + delta
        target = max(0, min(timeline_slider.maximum(), target))
        frame_state["suppress_slider_signal"] = True
        timeline_slider.setValue(target)
        frame_state["suppress_slider_signal"] = False
        _apply_slider_value(target)

    def on_video_label_resize(_event) -> None:
        if video_state["current_pixmap"] is None:
            return
        scaled = video_state["current_pixmap"].scaled(
            video_label.size(),
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation,
        )
        video_label.setPixmap(scaled)

    video_label.resizeEvent = on_video_label_resize

    QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key.Key_Space), main_window, activated=on_play_pause_clicked)
    QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key.Key_Left), main_window, activated=lambda: seek_by_frames(-1))
    QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key.Key_Right), main_window, activated=lambda: seek_by_frames(1))
    QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key.Key_Comma), main_window, activated=lambda: nudge_video_offset(-0.02))
    QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key.Key_Period), main_window, activated=lambda: nudge_video_offset(0.02))
    QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key.Key_Less), main_window, activated=lambda: nudge_video_offset(-0.10))
    QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key.Key_Greater), main_window, activated=lambda: nudge_video_offset(0.10))

    play_pause_button.clicked.connect(on_play_pause_clicked)
    timeline_slider.sliderPressed.connect(on_slider_pressed)
    timeline_slider.sliderMoved.connect(on_slider_moved)
    timeline_slider.sliderReleased.connect(on_slider_released)
    timeline_slider.valueChanged.connect(on_slider_changed)

    set_frame(0, update_slider=True, timeline_time_s=0.0)
    set_playing(True)

    timer = QtCore.QTimer()
    timer.timeout.connect(on_tick)
    timer.start(interval_ms_int)

    app.exec()
    video_capture.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
