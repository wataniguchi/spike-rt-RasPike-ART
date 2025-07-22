// SPDX-License-Identifier: MIT

/**
 * We define the IMU API by reusing lib/pbio/include/pbio/imu.h
 * from the snapshot of pybricks-micropython as of 2024/10/29 @accb117.
 *
 * Original codes Copyright (c) 2023 The Pybricks Authors
 * Modifications for TOPPERS/APS3 Kernel Copyright (c) 2022 Embedded and Real-Time Systems Laboratory,
 *                                                          Graduate School of Information Science, Nagoya Univ., JAPAN
 */

#ifndef __SPIKE_IMU_H__
#define __SPIKE_IMU_H__

#include <stdint.h>

#include <pbio/angle.h>
#include <pbio/error.h>
#include <pbio/geometry.h>

void imu_handle_frame_data_func(void);

void pbio_imu_set_configuration(const float angular_velocity_bias[3], const float angular_velocity_scale[3], const float acceleration_correction[6]);

pbio_error_t pbio_imu_set_base_orientation(pbio_geometry_xyz_t *x_axis, pbio_geometry_xyz_t *z_axis);

void pbio_imu_get_angular_velocity(pbio_geometry_xyz_t *values, bool calibrated);

void pbio_imu_get_acceleration(pbio_geometry_xyz_t *values, bool calibrated);

void pbio_imu_get_tilt_vector(pbio_geometry_xyz_t *values);

pbio_error_t pbio_imu_get_single_axis_rotation(pbio_geometry_xyz_t *axis, float *angle);

pbio_geometry_side_t pbio_imu_get_up_side(bool calibrated);

float pbio_imu_get_heading(void);

void pbio_imu_set_heading(float desired_heading);

void pbio_imu_get_heading_scaled(pbio_angle_t *heading, int32_t *heading_rate, int32_t ctl_steps_per_degree);

void pbio_orientation_imu_get_rotation(pbio_geometry_matrix_3x3_t *rotation);

#endif // __SPIKE_IMU_H__

/** @} */
