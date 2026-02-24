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


def _estimate_initial_gyro_bias_dps_quiet(
    *,
    sample_list: list,
    accel_scale: float,
    gyro_scale: float,
    sensor_to_body_matrix: list[list[float]],
) -> list[float]:
    if len(sample_list) < 2:
        return [0.0, 0.0, 0.0]

    start_ts = sample_list[0].timestamp_us
    window_start_ts = start_ts + 500_000
    window_end_ts = window_start_ts + 2_000_000
    accel_gate_g = 0.20

    gx_values: list[float] = []
    gy_values: list[float] = []
    gz_values: list[float] = []

    for sample in sample_list:
        ts = sample.timestamp_us
        if ts < window_start_ts or ts >= window_end_ts:
            continue

        accel_sensor_g = (
            sample.accel_x * accel_scale,
            sample.accel_y * accel_scale,
            sample.accel_z * accel_scale,
        )
        accel_body_g = mat3_mul_vec(sensor_to_body_matrix, accel_sensor_g)
        accel_mag_g = math.sqrt(
            accel_body_g[0] * accel_body_g[0]
            + accel_body_g[1] * accel_body_g[1]
            + accel_body_g[2] * accel_body_g[2]
        )
        if abs(accel_mag_g - 1.0) > accel_gate_g:
            continue

        gyro_sensor_dps = (
            sample.gyro_x * gyro_scale,
            sample.gyro_y * gyro_scale,
            sample.gyro_z * gyro_scale,
        )
        gyro_body_dps = mat3_mul_vec(sensor_to_body_matrix, gyro_sensor_dps)
        gx_values.append(gyro_body_dps[0])
        gy_values.append(gyro_body_dps[1])
        gz_values.append(gyro_body_dps[2])

    if len(gx_values) < 50:
        return [0.0, 0.0, 0.0]

    return [
        sum(gx_values) / len(gx_values),
        sum(gy_values) / len(gy_values),
        sum(gz_values) / len(gz_values),
    ]


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

    gravity_nominal_m_s2 = 9.80665
    still_accel_enter_g = 0.10
    still_accel_exit_g = 0.14
    still_gyro_enter_dps = 3.5
    still_gyro_exit_dps = 5.0
    bootstrap_accel_gate_g = 0.035

    accel_deadband_plane_m_s2 = 0.18
    velocity_leak_tau_plane_s = 3.0

    gyro_bias_tau_s = 8.0
    gyro_bias_tau_bootstrap_s = 2.0

    startup_settle_s = 0.8
    gravity_calib_max_s = 3.0
    gravity_calib_min_samples = 100

    velocity_m_s = [0.0, 0.0, 0.0]
    position_m = [0.0, 0.0, 0.0]
    gravity_world_estimate = [0.0, 0.0, gravity_nominal_m_s2]
    gravity_calib_samples = 0
    gyro_bias_dps = _estimate_initial_gyro_bias_dps_quiet(
        sample_list=sample_list,
        accel_scale=accel_scale,
        gyro_scale=gyro_scale,
        sensor_to_body_matrix=sensor_to_body_matrix,
    )
    still_state = False

    start_timestamp_us = sample_list[0].timestamp_us
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
        gyro_unbiased_dps = (
            gyro_body_dps[0] - gyro_bias_dps[0],
            gyro_body_dps[1] - gyro_bias_dps[1],
            gyro_body_dps[2] - gyro_bias_dps[2],
        )
        gyro_mag_dps = math.sqrt(
            gyro_unbiased_dps[0] * gyro_unbiased_dps[0]
            + gyro_unbiased_dps[1] * gyro_unbiased_dps[1]
            + gyro_unbiased_dps[2] * gyro_unbiased_dps[2]
        )

        if still_state:
            still_state = abs(accel_mag_g - 1.0) <= still_accel_exit_g and gyro_mag_dps <= still_gyro_exit_dps
        else:
            still_state = abs(accel_mag_g - 1.0) <= still_accel_enter_g and gyro_mag_dps <= still_gyro_enter_dps
        is_still = still_state

        elapsed_s = (timestamp_us - start_timestamp_us) / 1_000_000.0
        early_bootstrap_window = elapsed_s <= 5.0
        accel_only_still = abs(accel_mag_g - 1.0) <= bootstrap_accel_gate_g
        bias_update_allowed = is_still or (early_bootstrap_window and accel_only_still)

        if bias_update_allowed:
            tau = gyro_bias_tau_bootstrap_s if early_bootstrap_window else gyro_bias_tau_s
            bias_alpha = dt / (tau + dt)
            gyro_bias_dps[0] = (1.0 - bias_alpha) * gyro_bias_dps[0] + bias_alpha * gyro_body_dps[0]
            gyro_bias_dps[1] = (1.0 - bias_alpha) * gyro_bias_dps[1] + bias_alpha * gyro_body_dps[1]
            gyro_bias_dps[2] = (1.0 - bias_alpha) * gyro_bias_dps[2] + bias_alpha * gyro_body_dps[2]

        rotation3 = quat_to_rotation_matrix(orientation.quaternion)
        accel_body_m_s2 = (
            accel_body_g[0] * gravity_nominal_m_s2,
            accel_body_g[1] * gravity_nominal_m_s2,
            accel_body_g[2] * gravity_nominal_m_s2,
        )
        accel_world_m_s2 = rotate_vector(rotation3, accel_body_m_s2)

        in_gravity_calibration_window = elapsed_s <= gravity_calib_max_s
        if bias_update_allowed and in_gravity_calibration_window:
            alpha = dt / (0.8 + dt)
            gravity_world_estimate[0] = (1.0 - alpha) * gravity_world_estimate[0] + alpha * accel_world_m_s2[0]
            gravity_world_estimate[1] = (1.0 - alpha) * gravity_world_estimate[1] + alpha * accel_world_m_s2[1]
            gravity_world_estimate[2] = (1.0 - alpha) * gravity_world_estimate[2] + alpha * accel_world_m_s2[2]
            gravity_calib_samples += 1

        gravity_calibrated = gravity_calib_samples >= gravity_calib_min_samples
        translation_enabled = elapsed_s >= startup_settle_s and gravity_calibrated

        linear_world_m_s2 = [
            accel_world_m_s2[0] - gravity_world_estimate[0],
            accel_world_m_s2[1] - gravity_world_estimate[1],
            accel_world_m_s2[2] - gravity_world_estimate[2],
        ]

        gravity_norm = math.sqrt(
            gravity_world_estimate[0] * gravity_world_estimate[0]
            + gravity_world_estimate[1] * gravity_world_estimate[1]
            + gravity_world_estimate[2] * gravity_world_estimate[2]
        )
        if gravity_norm > 1e-6:
            gx_hat = gravity_world_estimate[0] / gravity_norm
            gy_hat = gravity_world_estimate[1] / gravity_norm
            gz_hat = gravity_world_estimate[2] / gravity_norm

            linear_along_gravity = (
                linear_world_m_s2[0] * gx_hat
                + linear_world_m_s2[1] * gy_hat
                + linear_world_m_s2[2] * gz_hat
            )

            linear_world_m_s2[0] -= linear_along_gravity * gx_hat
            linear_world_m_s2[1] -= linear_along_gravity * gy_hat
            linear_world_m_s2[2] -= linear_along_gravity * gz_hat

            linear_world_m_s2[0] = _apply_component_deadband(linear_world_m_s2[0], accel_deadband_plane_m_s2)
            linear_world_m_s2[1] = _apply_component_deadband(linear_world_m_s2[1], accel_deadband_plane_m_s2)
            linear_world_m_s2[2] = _apply_component_deadband(linear_world_m_s2[2], accel_deadband_plane_m_s2)

            if is_still or not translation_enabled:
                velocity_m_s = [0.0, 0.0, 0.0]
            else:
                velocity_m_s[0] += linear_world_m_s2[0] * dt
                velocity_m_s[1] += linear_world_m_s2[1] * dt
                velocity_m_s[2] += linear_world_m_s2[2] * dt

                leak = dt / (velocity_leak_tau_plane_s + dt)
                velocity_m_s[0] *= 1.0 - leak
                velocity_m_s[1] *= 1.0 - leak
                velocity_m_s[2] *= 1.0 - leak

                velocity_along_gravity = (
                    velocity_m_s[0] * gx_hat
                    + velocity_m_s[1] * gy_hat
                    + velocity_m_s[2] * gz_hat
                )
                velocity_m_s[0] -= velocity_along_gravity * gx_hat
                velocity_m_s[1] -= velocity_along_gravity * gy_hat
                velocity_m_s[2] -= velocity_along_gravity * gz_hat

            position_m[0] += velocity_m_s[0] * dt
            position_m[1] += velocity_m_s[1] * dt
            position_m[2] += velocity_m_s[2] * dt

            position_along_gravity = (
                position_m[0] * gx_hat
                + position_m[1] * gy_hat
                + position_m[2] * gz_hat
            )
            position_m[0] -= position_along_gravity * gx_hat
            position_m[1] -= position_along_gravity * gy_hat
            position_m[2] -= position_along_gravity * gz_hat

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
