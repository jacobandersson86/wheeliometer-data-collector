from __future__ import annotations

import math
from typing import Iterable

try:
    from analysis.orientation_fusion import (
        IDENTITY_MATRIX_3X3,
        PoseSample,
        fuse_orientation,
        mat3_mul_vec,
        quat_to_rotation_matrix,
        rotate_vector,
    )
except ImportError:
    from orientation_fusion import (
        IDENTITY_MATRIX_3X3,
        PoseSample,
        fuse_orientation,
        mat3_mul_vec,
        quat_to_rotation_matrix,
        rotate_vector,
    )


def _apply_component_deadband(value: float, threshold: float) -> float:
    if abs(value) <= threshold:
        return 0.0
    return value - math.copysign(threshold, value)


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

    orientation_samples = fuse_orientation(
        samples=sample_list,
        accel_fsr=accel_fsr,
        gyro_fsr=gyro_fsr,
        kp=kp,
        sensor_to_body_matrix=sensor_to_body_matrix,
    )

    accel_scale = accel_fsr / 32768.0
    gyro_scale = gyro_fsr / 32768.0

    gravity_m_s2 = 9.80665
    still_accel_gate_g = 0.07
    still_gyro_gate_dps = 1.2
    accel_deadband_m_s2 = 0.15
    velocity_leak_tau_s = 2.0
    horizontal_bias_tau_s = 1.2

    velocity_m_s = [0.0, 0.0, 0.0]
    position_m = [0.0, 0.0, 0.0]
    horizontal_bias_m_s2 = [0.0, 0.0]
    previous_timestamp_us: int | None = None

    pose_samples: list[PoseSample] = []

    for sample, orientation in zip(sample_list, orientation_samples):
        timestamp_us = sample.timestamp_us

        if previous_timestamp_us is None:
            previous_timestamp_us = timestamp_us
            pose_samples.append(
                PoseSample(
                    timestamp_us=timestamp_us,
                    quaternion=orientation.quaternion,
                    position_m=(0.0, 0.0, 0.0),
                    velocity_m_s=(0.0, 0.0, 0.0),
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
                    quaternion=orientation.quaternion,
                    position_m=(position_m[0], position_m[1], position_m[2]),
                    velocity_m_s=(velocity_m_s[0], velocity_m_s[1], velocity_m_s[2]),
                    is_still=False,
                )
            )
            continue

        if dt > 0.05:
            dt = 0.05

        accel_sensor_g = (
            sample.accel_x * accel_scale,
            sample.accel_y * accel_scale,
            sample.accel_z * accel_scale,
        )
        gyro_sensor_dps = (
            sample.gyro_x * gyro_scale,
            sample.gyro_y * gyro_scale,
            sample.gyro_z * gyro_scale,
        )

        accel_body_g = mat3_mul_vec(sensor_to_body_matrix, accel_sensor_g)
        gyro_body_dps = mat3_mul_vec(sensor_to_body_matrix, gyro_sensor_dps)

        accel_mag_g = math.sqrt(
            accel_body_g[0] * accel_body_g[0]
            + accel_body_g[1] * accel_body_g[1]
            + accel_body_g[2] * accel_body_g[2]
        )
        gyro_mag_dps = math.sqrt(
            gyro_body_dps[0] * gyro_body_dps[0]
            + gyro_body_dps[1] * gyro_body_dps[1]
            + gyro_body_dps[2] * gyro_body_dps[2]
        )

        is_still = abs(accel_mag_g - 1.0) <= still_accel_gate_g and gyro_mag_dps <= still_gyro_gate_dps

        rotation3 = quat_to_rotation_matrix(orientation.quaternion)
        accel_body_m_s2 = (
            accel_body_g[0] * gravity_m_s2,
            accel_body_g[1] * gravity_m_s2,
            accel_body_g[2] * gravity_m_s2,
        )
        accel_world_m_s2 = rotate_vector(rotation3, accel_body_m_s2)

        horizontal_linear_m_s2 = [
            accel_world_m_s2[0],
            accel_world_m_s2[1],
        ]

        if is_still:
            bias_alpha = dt / (horizontal_bias_tau_s + dt)
            horizontal_bias_m_s2[0] = (1.0 - bias_alpha) * horizontal_bias_m_s2[0] + bias_alpha * horizontal_linear_m_s2[0]
            horizontal_bias_m_s2[1] = (1.0 - bias_alpha) * horizontal_bias_m_s2[1] + bias_alpha * horizontal_linear_m_s2[1]

        horizontal_linear_m_s2[0] -= horizontal_bias_m_s2[0]
        horizontal_linear_m_s2[1] -= horizontal_bias_m_s2[1]

        horizontal_linear_m_s2[0] = _apply_component_deadband(horizontal_linear_m_s2[0], accel_deadband_m_s2)
        horizontal_linear_m_s2[1] = _apply_component_deadband(horizontal_linear_m_s2[1], accel_deadband_m_s2)

        if is_still:
            velocity_m_s = [0.0, 0.0, 0.0]
        else:
            velocity_m_s[0] += horizontal_linear_m_s2[0] * dt
            velocity_m_s[1] += horizontal_linear_m_s2[1] * dt

            leak_alpha = dt / (velocity_leak_tau_s + dt)
            velocity_m_s[0] *= 1.0 - leak_alpha
            velocity_m_s[1] *= 1.0 - leak_alpha
            velocity_m_s[2] = 0.0

        position_m[0] += velocity_m_s[0] * dt
        position_m[1] += velocity_m_s[1] * dt
        position_m[2] = 0.0

        pose_samples.append(
            PoseSample(
                timestamp_us=timestamp_us,
                quaternion=orientation.quaternion,
                position_m=(position_m[0], position_m[1], position_m[2]),
                velocity_m_s=(velocity_m_s[0], velocity_m_s[1], velocity_m_s[2]),
                is_still=is_still,
            )
        )

    return pose_samples
