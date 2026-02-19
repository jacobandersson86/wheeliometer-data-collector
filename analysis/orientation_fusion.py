from __future__ import annotations

from dataclasses import dataclass
import math
import sys
from typing import Iterable

import numpy as np

IDENTITY_MATRIX_3X3: list[list[float]] = [
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
]


@dataclass(frozen=True)
class OrientationSample:
    timestamp_us: int
    quaternion: list[float]


@dataclass(frozen=True)
class PoseSample:
    timestamp_us: int
    quaternion: list[float]
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    is_still: bool


@dataclass(frozen=True)
class BiasEstimate:
    gx_bias_rad_s: float
    gy_bias_rad_s: float
    gz_bias_rad_s: float
    used_sample_count: int
    total_window_sample_count: int
    still_fraction: float


def quat_normalize(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(component * component for component in q))
    if norm == 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [component / norm for component in q]


def quat_multiply(a: list[float], b: list[float]) -> list[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def quat_to_rotation_matrix(q: list[float]) -> list[list[float]]:
    w, x, y, z = quat_normalize(q)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rotate_vector(rotation: list[list[float]], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    vx, vy, vz = vector
    return (
        rotation[0][0] * vx + rotation[0][1] * vy + rotation[0][2] * vz,
        rotation[1][0] * vx + rotation[1][1] * vy + rotation[1][2] * vz,
        rotation[2][0] * vx + rotation[2][1] * vy + rotation[2][2] * vz,
    )


def mat3_mul_vec(matrix: list[list[float]], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vector
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


def parse_sensor_to_body_matrix(raw_value: str | None) -> list[list[float]]:
    if raw_value is None:
        return [row[:] for row in IDENTITY_MATRIX_3X3]

    parts = [part.strip() for part in raw_value.split(",") if part.strip()]
    if len(parts) != 9:
        raise ValueError(
            "--sensor-to-body must have 9 comma-separated values in row-major order "
            "(r11,r12,r13,r21,r22,r23,r31,r32,r33)"
        )

    values = [float(part) for part in parts]
    return [
        values[0:3],
        values[3:6],
        values[6:9],
    ]


def validate_sensor_to_body_matrix(matrix: list[list[float]], tolerance: float = 1e-2) -> tuple[bool, str]:
    rows = [np.array(row, dtype=np.float64) for row in matrix]
    row_norms = [float(np.linalg.norm(row)) for row in rows]
    dot01 = float(np.dot(rows[0], rows[1]))
    dot02 = float(np.dot(rows[0], rows[2]))
    dot12 = float(np.dot(rows[1], rows[2]))
    det = float(np.linalg.det(np.array(matrix, dtype=np.float64)))

    norms_ok = all(abs(norm - 1.0) <= tolerance for norm in row_norms)
    orthogonal_ok = abs(dot01) <= tolerance and abs(dot02) <= tolerance and abs(dot12) <= tolerance
    det_ok = abs(det - 1.0) <= 5e-2

    if norms_ok and orthogonal_ok and det_ok:
        return True, "ok"

    return (
        False,
        "Matrix may not be a proper rotation (sensor→body). "
        f"row_norms={row_norms}, dots=({dot01:.3f},{dot02:.3f},{dot12:.3f}), det={det:.3f}",
    )


def magnitude3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = _norm(v)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def estimate_initial_gyro_bias(
    samples: list,
    accel_scale: float,
    gyro_scale: float,
    sensor_to_body_matrix: list[list[float]],
) -> BiasEstimate | None:
    if len(samples) < 2:
        return None

    start_ts = samples[0].timestamp_us
    window_start_ts = start_ts + 500_000
    window_end_ts = window_start_ts + 2_000_000

    window_samples = [sample for sample in samples if window_start_ts <= sample.timestamp_us < window_end_ts]
    if not window_samples:
        print(
            "WARNING: Startup auto-bias window has no samples (expected 0.5s..2.5s). "
            "Continuing without startup gyro bias.",
            file=sys.stderr,
        )
        return None

    accel_gate_g = 0.20
    accel_gated_samples = []
    accel_magnitudes = []

    for sample in window_samples:
        accel_sensor = (
            sample.accel_x * accel_scale,
            sample.accel_y * accel_scale,
            sample.accel_z * accel_scale,
        )
        ax_g, ay_g, az_g = mat3_mul_vec(sensor_to_body_matrix, accel_sensor)
        accel_mag_g = magnitude3(ax_g, ay_g, az_g)
        if abs(accel_mag_g - 1.0) <= accel_gate_g:
            accel_gated_samples.append(sample)
            accel_magnitudes.append(accel_mag_g)

    if len(accel_gated_samples) < 50:
        print(
            "WARNING: Startup auto-bias quality is poor: "
            f"only {len(accel_gated_samples)}/{len(window_samples)} samples near 1g in first 2s window after 500ms. "
            "Continuing without startup gyro bias.",
            file=sys.stderr,
        )
        return None

    gyro_body_values = [
        mat3_mul_vec(
            sensor_to_body_matrix,
            (
                sample.gyro_x * gyro_scale,
                sample.gyro_y * gyro_scale,
                sample.gyro_z * gyro_scale,
            ),
        )
        for sample in accel_gated_samples
    ]
    gx_dps_values = [value[0] for value in gyro_body_values]
    gy_dps_values = [value[1] for value in gyro_body_values]
    gz_dps_values = [value[2] for value in gyro_body_values]

    def _mean(values: list[float]) -> float:
        return sum(values) / len(values)

    def _std(values: list[float]) -> float:
        if len(values) <= 1:
            return 0.0
        m = _mean(values)
        variance = sum((value - m) * (value - m) for value in values) / len(values)
        return math.sqrt(variance)

    gx_mean_dps = _mean(gx_dps_values)
    gy_mean_dps = _mean(gy_dps_values)
    gz_mean_dps = _mean(gz_dps_values)

    gx_std_dps = _std(gx_dps_values)
    gy_std_dps = _std(gy_dps_values)
    gz_std_dps = _std(gz_dps_values)
    gyro_std_mag_dps = magnitude3(gx_std_dps, gy_std_dps, gz_std_dps)
    accel_std_g = _std(accel_magnitudes)

    if gyro_std_mag_dps > 4.0 or accel_std_g > 0.10:
        print(
            "WARNING: Startup auto-bias window does not look still enough: "
            f"gyro_std_mag={gyro_std_mag_dps:.2f} dps, accel_std={accel_std_g:.3f} g. "
            "Continuing without startup gyro bias.",
            file=sys.stderr,
        )
        return None

    if gyro_std_mag_dps > 1.5 or accel_std_g > 0.05:
        print(
            "WARNING: Startup auto-bias quality is moderate: "
            f"gyro_std_mag={gyro_std_mag_dps:.2f} dps, accel_std={accel_std_g:.3f} g. "
            "Bias is still applied.",
            file=sys.stderr,
        )

    total_count = len(window_samples)
    used_count = len(accel_gated_samples)
    still_fraction = used_count / total_count if total_count > 0 else 0.0

    if still_fraction < 0.65:
        print(
            "WARNING: Startup auto-bias window appears not fully still: "
            f"near-1g fraction={still_fraction:.2f} ({used_count}/{total_count}). "
            "Bias is estimated from near-1g samples only.",
            file=sys.stderr,
        )

    gx_bias = math.radians(gx_mean_dps)
    gy_bias = math.radians(gy_mean_dps)
    gz_bias = math.radians(gz_mean_dps)

    print(
        "Startup gyro bias estimated (dps): "
        f"gx={gx_mean_dps:.4f}, gy={gy_mean_dps:.4f}, gz={gz_mean_dps:.4f} "
        f"using {used_count}/{total_count} near-1g samples",
        file=sys.stderr,
    )

    return BiasEstimate(
        gx_bias_rad_s=gx_bias,
        gy_bias_rad_s=gy_bias,
        gz_bias_rad_s=gz_bias,
        used_sample_count=used_count,
        total_window_sample_count=total_count,
        still_fraction=still_fraction,
    )


def fuse_orientation(
    samples: Iterable,
    accel_fsr: int,
    gyro_fsr: int,
    kp: float = 1.8,
    sensor_to_body_matrix: list[list[float]] | None = None,
) -> list[OrientationSample]:
    pose_samples = fuse_pose(
        samples=samples,
        accel_fsr=accel_fsr,
        gyro_fsr=gyro_fsr,
        kp=kp,
        sensor_to_body_matrix=sensor_to_body_matrix,
    )
    return [OrientationSample(timestamp_us=sample.timestamp_us, quaternion=sample.quaternion) for sample in pose_samples]


def fuse_pose(
    samples: Iterable,
    accel_fsr: int,
    gyro_fsr: int,
    kp: float = 1.8,
    sensor_to_body_matrix: list[list[float]] | None = None,
) -> list[PoseSample]:
    sample_list = list(samples)
    if not sample_list:
        return []

    if sensor_to_body_matrix is None:
        sensor_to_body_matrix = [row[:] for row in IDENTITY_MATRIX_3X3]

    accel_scale = accel_fsr / 32768.0
    gyro_scale = gyro_fsr / 32768.0

    pose_samples: list[PoseSample] = []
    q = [1.0, 0.0, 0.0, 0.0]
    position_m = (0.0, 0.0, 0.0)
    velocity_m_s = (0.0, 0.0, 0.0)
    previous_timestamp_us: int | None = None

    initial_bias = estimate_initial_gyro_bias(
        sample_list,
        accel_scale=accel_scale,
        gyro_scale=gyro_scale,
        sensor_to_body_matrix=sensor_to_body_matrix,
    )
    if initial_bias is None:
        gyro_bias_x = 0.0
        gyro_bias_y = 0.0
        gyro_bias_z = 0.0
    else:
        gyro_bias_x = initial_bias.gx_bias_rad_s
        gyro_bias_y = initial_bias.gy_bias_rad_s
        gyro_bias_z = initial_bias.gz_bias_rad_s

    accel_gate_g = 0.15
    still_accel_gate_g = 0.08
    still_gyro_gate_dps = 1.5
    bias_tau_s = 8.0
    gravity_m_s2 = 9.80665

    for sample in sample_list:
        timestamp_us = sample.timestamp_us

        if previous_timestamp_us is None:
            previous_timestamp_us = timestamp_us
            pose_samples.append(
                PoseSample(
                    timestamp_us=timestamp_us,
                    quaternion=q.copy(),
                    position_m=position_m,
                    velocity_m_s=velocity_m_s,
                    is_still=True,
                )
            )
            continue

        dt = (timestamp_us - previous_timestamp_us) / 1_000_000.0
        previous_timestamp_us = timestamp_us

        if dt <= 0.0:
            pose_samples.append(
                PoseSample(
                    timestamp_us=timestamp_us,
                    quaternion=q.copy(),
                    position_m=position_m,
                    velocity_m_s=velocity_m_s,
                    is_still=False,
                )
            )
            continue

        if dt > 0.05:
            dt = 0.05

        gyro_sensor_dps = (
            sample.gyro_x * gyro_scale,
            sample.gyro_y * gyro_scale,
            sample.gyro_z * gyro_scale,
        )
        gyro_body_dps = mat3_mul_vec(sensor_to_body_matrix, gyro_sensor_dps)
        gx_raw = math.radians(gyro_body_dps[0])
        gy_raw = math.radians(gyro_body_dps[1])
        gz_raw = math.radians(gyro_body_dps[2])

        accel_sensor_g = (
            sample.accel_x * accel_scale,
            sample.accel_y * accel_scale,
            sample.accel_z * accel_scale,
        )
        ax, ay, az = mat3_mul_vec(sensor_to_body_matrix, accel_sensor_g)

        accel_mag_g = magnitude3(ax, ay, az)

        gx = gx_raw - gyro_bias_x
        gy = gy_raw - gyro_bias_y
        gz = gz_raw - gyro_bias_z

        gyro_mag_dps = math.degrees(magnitude3(gx, gy, gz))
        is_still = gyro_mag_dps <= still_gyro_gate_dps and abs(accel_mag_g - 1.0) <= still_accel_gate_g

        if is_still:
            alpha = dt / (bias_tau_s + dt)
            gyro_bias_x = (1.0 - alpha) * gyro_bias_x + alpha * gx_raw
            gyro_bias_y = (1.0 - alpha) * gyro_bias_y + alpha * gy_raw
            gyro_bias_z = (1.0 - alpha) * gyro_bias_z + alpha * gz_raw

            gx = gx_raw - gyro_bias_x
            gy = gy_raw - gyro_bias_y
            gz = gz_raw - gyro_bias_z

            if gyro_mag_dps < 0.2:
                gx = 0.0
                gy = 0.0
                gz = 0.0

        measured_gravity = _normalize((ax, ay, az))
        use_accel_correction = abs(accel_mag_g - 1.0) <= accel_gate_g
        if use_accel_correction and measured_gravity != (0.0, 0.0, 0.0):
            rotation = quat_to_rotation_matrix(q)
            predicted_gravity = _normalize((rotation[0][2], rotation[1][2], rotation[2][2]))
            error = _cross(predicted_gravity, measured_gravity)
            gx += kp * error[0]
            gy += kp * error[1]
            gz += kp * error[2]

        q_dot = quat_multiply(q, [0.0, gx, gy, gz])
        q = [
            q[0] + 0.5 * q_dot[0] * dt,
            q[1] + 0.5 * q_dot[1] * dt,
            q[2] + 0.5 * q_dot[2] * dt,
            q[3] + 0.5 * q_dot[3] * dt,
        ]
        q = quat_normalize(q)

        accel_body_m_s2 = (
            ax * gravity_m_s2,
            ay * gravity_m_s2,
            az * gravity_m_s2,
        )
        rotation = quat_to_rotation_matrix(q)
        accel_world_m_s2 = rotate_vector(rotation, accel_body_m_s2)
        linear_world_m_s2 = (
            accel_world_m_s2[0],
            accel_world_m_s2[1],
            accel_world_m_s2[2] - gravity_m_s2,
        )

        if is_still:
            velocity_m_s = (0.0, 0.0, 0.0)
        else:
            velocity_m_s = (
                velocity_m_s[0] + linear_world_m_s2[0] * dt,
                velocity_m_s[1] + linear_world_m_s2[1] * dt,
                velocity_m_s[2] + linear_world_m_s2[2] * dt,
            )

        position_m = (
            position_m[0] + velocity_m_s[0] * dt,
            position_m[1] + velocity_m_s[1] * dt,
            position_m[2] + velocity_m_s[2] * dt,
        )

        pose_samples.append(
            PoseSample(
                timestamp_us=timestamp_us,
                quaternion=q.copy(),
                position_m=position_m,
                velocity_m_s=velocity_m_s,
                is_still=is_still,
            )
        )

    return pose_samples


def rotation_matrix_4x4(rotation3: list[list[float]]) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, :3] = np.array(rotation3, dtype=np.float32)
    return matrix


def pose_matrix_4x4(rotation3: list[list[float]], translation_m: tuple[float, float, float]) -> np.ndarray:
    matrix = rotation_matrix_4x4(rotation3)
    matrix[0, 3] = translation_m[0]
    matrix[1, 3] = translation_m[1]
    matrix[2, 3] = translation_m[2]
    return matrix


def select_frame_indices(orientation: list[OrientationSample], render_fps: float) -> list[int]:
    if len(orientation) <= 1:
        return list(range(len(orientation)))

    start_us = orientation[0].timestamp_us
    end_us = orientation[-1].timestamp_us
    if end_us <= start_us:
        return list(range(len(orientation)))

    frame_dt_us = int(1_000_000 / render_fps)
    frame_indices: list[int] = []
    sample_index = 0
    target_ts = start_us

    while target_ts <= end_us and sample_index < len(orientation):
        while sample_index + 1 < len(orientation) and orientation[sample_index + 1].timestamp_us <= target_ts:
            sample_index += 1
        frame_indices.append(sample_index)
        target_ts += frame_dt_us

    if frame_indices[-1] != len(orientation) - 1:
        frame_indices.append(len(orientation) - 1)

    return frame_indices
