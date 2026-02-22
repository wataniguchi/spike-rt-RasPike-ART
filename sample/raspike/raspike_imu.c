// SPDX-License-Identifier: MIT
/**
 * We implement the IMU API by reusing:
 *   lib/pbio/src/imu.c
 *   lib/pbio/drv/imu/imu_lsm6ds3tr_c_stm32.c 
 * from the snapshot of pybricks-micropython as of 2025/02/25 @57793e3.
 *
 * Original codes Copyright (c) 2022-2023 The Pybricks Authors
 * Modifications for TOPPERS/APS3 Kernel Copyright (c) 2022 Embedded and Real-Time Systems Laboratory,
 *                                                          Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 * The following files have been copied from the same snapshot:
 *   lib/pbio/include/pbio/geometry.h -> sample/raspike/pbio/geometry.h
 *   lib/pbio/src/geometry.c -> sample/raspike/pbio/geometry.c
*/
#include <kernel.h>
#include "kernel_cfg.h"
#include <t_syslog.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <math.h>

#include <pbio/util.h>

#include <spike/hub/imu.h>
#include "raspike_imu.h"

/**
 * User Configuration Values
 *
 * These values should be determined for each hub by doing a calibration procedure
 * and are set by invoking raspike_imu_initilize().
 * For details of IMU calibration, refer to the following post:
 * https://github.com/pybricks/support/issues/1907
 */
bool raspike_imu_initialized = false;
/** Angular velocity threshold below which the IMU is considered stationary. */
float config_gyro_stationary_threshold;
/** Acceleration threshold below which the IMU is considered stationary. */
float config_accel_stationary_threshold;
/** Angular velocity scale (unadjusted measured degrees per whole rotation) */
pbio_geometry_xyz_t config_angular_velocity_scale;
/** Positive acceleration values */
pbio_geometry_xyz_t config_gravity_pos;
/** Negative acceleration values */
pbio_geometry_xyz_t config_gravity_neg;


/**
 * Uncalibrated angular velocity in the hub frame.
 *
 * These are scaled from raw units to degrees per second using only the
 * datasheet/hal conversion constant, but otherwise not further adjusted.
 */
pbio_geometry_xyz_t angular_velocity_uncalibrated;

/**
 * Estimated gyro bias value in degrees per second.
 *
 * This is a measure for the uncalibrated angular velocity above, averaged over
 * time. If specified, the value starts at the last saved user value, then
 * updates over time.
 */
pbio_geometry_xyz_t gyro_bias;

/**
 * Calibrated angular velocity in the hub frame degrees per second.
 *
 * This takes the uncalibrated value above, subtracts the bias estimate, and
 * rescales by a user calibration factor to ensure that integrating over one
 * full rotation adds up to 360 degrees.
 */
pbio_geometry_xyz_t angular_velocity_calibrated;

/**
 * Uncalibrated acceleration in the hub frame in mm/s^2.
 *
 * These are scaled from raw units to mm/s^2 using only the
 * datasheet/hal conversion constant, but otherwise not further adjusted.
 */
pbio_geometry_xyz_t acceleration_uncalibrated;

/**
 * Calibrated acceleration in the hub frame mm/s^2.
 *
 * This takes the uncalibrated value above, and subtracts a constant user offset
 * and scales by a previously determined user factor to normalize to gravity magnitude.
 */
pbio_geometry_xyz_t acceleration_calibrated; // mm/s^2, in hub frame

/**
 * 1D integrated angular velocity for each body axis.
 *
 * This is based on integrating the calibrated angular velocity over time, so
 * including its bias and adjustments to achieve 360 degrees per rotation.
 *
 * This is not used for 3D attitude estimation, but serves as a useful way to
 * estimate 1D rotations without being effected by accelerometer fusion which
 * may leads to unwanted adjustments in applications like balancing robots.
 */
pbio_geometry_xyz_t single_axis_rotation; // deg, in hub frame

/**
 * Rotation of the hub with respect to the inertial frame, see R(q) below.
 *
 * Initialized as the identity quaternion. Updated on first gravity sample.
 */
pbio_geometry_quaternion_t quaternion = {
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,
    .q4 = 1.0f,
};

/**
 * Flag to indicate if the quaternion has been initialized to the very first
 * gravity sample.
 */
bool quaternion_initialized = false;

/**
 * Rotation of the hub with respect to the inertial frame.
 *
 * Does *not* use the user application frame.
 *
 * The matrix R(q) is defined such that it transforms hub body frame vectors to
 * vectors in the inertial frame as:
 *
 *    v_inertial = R(q) * v_body
 *
 * Initialized as the identity matrix. Must match initial value of quaternion.
 */
pbio_geometry_matrix_3x3_t pbio_imu_rotation = {
    .m11 = 1.0f, .m12 = 0.0f, .m13 = 0.0f,
    .m21 = 0.0f, .m22 = 1.0f, .m23 = 0.0f,
    .m31 = 0.0f, .m32 = 0.0f, .m33 = 1.0f,
};


/**
 * The "neutral" base orientation of the hub, describing how it is mounted
 * in the robot. All getters (tilt, acceleration, rotation, etc) give results
 * relative to this base orientation:
 *
 * vector_reported = R_base * vector_in_hub_body_frame
 *
 * Default orientation is identity, hub flat.
 */
pbio_geometry_matrix_3x3_t pbio_imu_base_orientation = {
    .m11 = 1.0f, .m12 = 0.0f, .m13 = 0.0f,
    .m21 = 0.0f, .m22 = 1.0f, .m23 = 0.0f,
    .m31 = 0.0f, .m32 = 0.0f, .m33 = 1.0f,
};

/**
 * The heading is defined as follows.
 *
 * Take the x-axis (after transformation to application frame) and project
 * into the inertial frame. Then project onto the horizontal (X-Y) plane. Then
 * take the angle between the projection and the x-axis, counterclockwise
 * positive.
 *
 * In practice, this means that when you look at a robot from the top, it is
 * the angle that its "forward direction vector" makes with respect to the
 * x-axis, even when the robot isn't perfectly flat.
 *
 */
float heading_projection;

/**
 * When the heading_projection flips from 180 to -180 or vice versa, we
 * increment or decrement the overal rotation counter to maintain a continuous
 * heading.
 */
int32_t heading_rotations;


/**
 * Standard gravity in mm/s^2.
 */
const float standard_gravity = 9806.65f;


/**
 * Given current orientation matrix, update the heading projection.
 *
 * This is called from the update loop so we can catch the projection jumping
 * across the 180/-180 boundary, and increment or decrement the rotation to
 * have a continuous heading.
 *
 * This is also called when the orientation frame is changed because this sets
 * the application x-axis used for the heading projection.
 */
static void update_heading_projection(void) {

    // Transform application x axis back into the hub frame (R_base^T * x_unit).
    pbio_geometry_xyz_t x_application = {
        .x = pbio_imu_base_orientation.m11,
        .y = pbio_imu_base_orientation.m12,
        .z = pbio_imu_base_orientation.m13
    };

    // Transform application x axis into the inertial frame via quaternion matrix.
    pbio_geometry_xyz_t x_inertial;
    pbio_geometry_vector_map(&pbio_imu_rotation, &x_application, &x_inertial);

    // Project onto the horizontal plane and use atan2 to get the angle.
    float heading_now = pbio_geometry_radians_to_degrees(atan2f(-x_inertial.y, x_inertial.x));

    // Update full rotation counter if the projection jumps across the 180/-180 boundary.
    if (heading_now < -90 && heading_projection > 90) {
        heading_rotations++;
    } else if (heading_now > 90 && heading_projection < -90) {
        heading_rotations--;
    }
    heading_projection = heading_now;
}

// This counter is a measure for calibration accuracy, roughly equivalent
// to the accumulative number of seconds it has been stationary in total.
uint32_t stationary_counter = 0;
SYSTIM stationary_time_last;

/*
 * Tests if the imu is ready for use in a user program.
 *
 * @return    True if it has been stationary at least once in the last 10 minutes.
*/
bool raspike_imu_is_ready(void) {
    SYSTIM current_time;
    get_tim(&current_time);
    return stationary_counter > 0 && current_time - stationary_time_last < 10 * 60 * 1000000;
}

typedef union {
    float data[6];
    struct {
        float gyro[3];
        float accel[3];
    };
} _raspike_imu_raw_data;

/** Most recent slow moving average of raw data. */
float data_slow[6];
/** Sum of raw data for slow moving average. */
float data_slow_sum[6];
/** Raw data count used for slow moving average. */
int32_t data_slow_count;
/** Start time of window in which stationary samples are recorded (us)*/
SYSTIM stationary_time_start;
/** Raw data point to which new samples are compared to detect stationary. */
float stationary_data_start[6];
/** Sum of gyro samples during the stationary period. */
float stationary_gyro_data_sum[3];
/** Sum of accelerometer samples during the stationary period. */
float stationary_accel_data_sum[3];
/** Number of sequential stationary samples. */
uint32_t stationary_sample_count;
/** Whether it is currently stationary, to be polled by higher level APIs. */
bool stationary_now;

static inline bool is_bounded(float diff, float threshold) {
    return diff < threshold && diff > -threshold;
}

static void reset_stationary_buffer(void) {
    stationary_sample_count = 0;
    get_tim(&stationary_time_start);
    for (uint8_t i = 0; i < 3; i++) {
        stationary_gyro_data_sum[i] = 0;
        stationary_accel_data_sum[i] = 0;
    }
}

static void update_slow_moving_average(_raspike_imu_raw_data *imu_data) {
    for (uint8_t i = 0; i < 6; i++) {
        data_slow_sum[i] += imu_data->data[i];
    }
    data_slow_count++;
    if (data_slow_count == 125) {
        for (uint8_t i = 0; i < 6; i++) {
            data_slow[i] = data_slow_sum[i] / data_slow_count;
            data_slow_sum[i] = 0;
        }
        data_slow_count = 0;
    }
}

static void update_stationary_status(_raspike_imu_raw_data *imu_data) {

    // Update slow moving average of raw data, used as starting point for stationary detection.
    update_slow_moving_average(imu_data);

    // Check whether still stationary compared to constant start sample.
    if (!is_bounded(imu_data->data[0] - stationary_data_start[0], config_gyro_stationary_threshold) ||
        !is_bounded(imu_data->data[1] - stationary_data_start[1], config_gyro_stationary_threshold) ||
        !is_bounded(imu_data->data[2] - stationary_data_start[2], config_gyro_stationary_threshold) ||
        !is_bounded(imu_data->data[3] - stationary_data_start[3], config_accel_stationary_threshold) ||
        !is_bounded(imu_data->data[4] - stationary_data_start[4], config_accel_stationary_threshold) ||
        !is_bounded(imu_data->data[5] - stationary_data_start[5], config_accel_stationary_threshold)
        ) {
        // Not stationary anymore, so reset counter and gyro sum data so we can start over.
        stationary_now = false;

        // Slow moving average becomes new starting value to compare to.
        for (uint8_t i = 0; i < 6; i++) {
            stationary_data_start[i] = data_slow[i];
        }

        reset_stationary_buffer();
        return;
    }

    // Updating running sum of stationary data.
    stationary_sample_count++;
    stationary_gyro_data_sum[0] += imu_data->data[0];
    stationary_gyro_data_sum[1] += imu_data->data[1];
    stationary_gyro_data_sum[2] += imu_data->data[2];
    stationary_accel_data_sum[0] += imu_data->data[3];
    stationary_accel_data_sum[1] += imu_data->data[4];
    stationary_accel_data_sum[2] += imu_data->data[5];

    // Exit if we don't have enough samples yet.
    if (stationary_sample_count < 1000) { // 1 sec worth of data
        return;
    }

    // This tells external APIs that we are really stationary.
    stationary_now = true;

    // If the IMU calibration hasn't been updated in a long time, reset the
    // stationary counter so that the calibration values get a large weight.
    if (!raspike_imu_is_ready()) {
        stationary_counter = 0;
    }

    get_tim(&stationary_time_last);
    stationary_counter++;

    // The relative weight of the new data in order to build a long term
    // average of the data without maintaining a data buffer.
    float weight = stationary_counter >= 20 ? 0.05f : 1.0f / stationary_counter;

    for (uint8_t i = 0; i < PBIO_ARRAY_SIZE(gyro_bias.values); i++) {
        // Average gyro rate while stationary, indicating current bias.
        float average_now = stationary_gyro_data_sum[i] / stationary_sample_count;

        // Update bias at decreasing rate.
        gyro_bias.values[i] = gyro_bias.values[i] * (1.0f - weight) + weight * average_now;
    }

    // Reset counter and gyro sum data so we can start over.
    reset_stationary_buffer();
}

// Called by periodic handler to process one frame of unfiltered gyro and accelerometer data.
void raspike_imu_handle_frame_data(uint32_t periodic_task_interval) {
    if (!raspike_imu_initialized) return; // until config set, do nothing

    _raspike_imu_raw_data imu_data;
    hub_imu_get_angular_velocity(imu_data.gyro); // raw angular velocity reading from IMU
    hub_imu_get_acceleration(imu_data.accel); // raw acceleration reading from IMU

    // Initialize quaternion from first gravity sample as a best-effort estimate.
    // From here, fusion will gradually converge the quaternion to the true value.
    if (!quaternion_initialized) {
        pbio_geometry_xyz_t g = { .x = imu_data.accel[0], .y = imu_data.accel[1], .z = imu_data.accel[2]};
        pbio_error_t err = pbio_geometry_vector_normalize(&g, &g);
        if (err != PBIO_SUCCESS) {
            // First sample not suited, try again on next sample.
            return;
        }
        pbio_geometry_quaternion_from_gravity_unit_vector(&g, &quaternion);
        quaternion_initialized = true;
    }

    // Compute current orientation matrix to obtain the current heading.
    pbio_geometry_quaternion_to_rotation_matrix(&quaternion, &pbio_imu_rotation);

    // Projects application x-axis into the inertial frame to compute the heading.
    update_heading_projection();

    for (uint8_t i = 0; i < PBIO_ARRAY_SIZE(angular_velocity_calibrated.values); i++) {
        // Update angular velocity and acceleration cache so user can read them.
        angular_velocity_uncalibrated.values[i] = imu_data.gyro[i];
        acceleration_uncalibrated.values[i] = imu_data.accel[i];

        // Maintain calibrated cached values.
        float acceleration_offset = (config_gravity_pos.values[i] + config_gravity_neg.values[i]) / 2;
        float acceleration_scale = (config_gravity_pos.values[i] - config_gravity_neg.values[i]) / 2;
        acceleration_calibrated.values[i] = (acceleration_uncalibrated.values[i] - acceleration_offset) * standard_gravity / acceleration_scale;
        angular_velocity_calibrated.values[i] = (angular_velocity_uncalibrated.values[i] - gyro_bias.values[i]) * 360.0f / config_angular_velocity_scale.values[i];

        // Update "heading" on all axes. This is not useful for 3D attitude
        // estimation, but it allows the user to get a 1D heading even with
        // the hub mounted at an arbitrary orientation. Such a 1D heading
        // is numerically more accurate, which is useful in drive base
        // applications so long as the vehicle drives on a flat surface.
        single_axis_rotation.values[i] += angular_velocity_calibrated.values[i] * periodic_task_interval / 1000000;
    }

    // Estimate for gravity vector based on orientation estimate.
    pbio_geometry_xyz_t s = {
        .x = pbio_imu_rotation.m31,
        .y = pbio_imu_rotation.m32,
        .z = pbio_imu_rotation.m33,
    };

    // We would like to adjust the attitude such that the gravity estimate
    // converges to the gravity value in the stationary case. If we subtract
    // both vectors we get the required direction of changes. This can be
    // thought of as a virtual spring between both vectors. This produces a
    // moment about the origin, which ultimately simplies to the following,
    // which we inject to the attitude integration.
    pbio_geometry_xyz_t correction;
    pbio_geometry_vector_cross_product(&s, &acceleration_calibrated, &correction);

    // Qualitative measures for how far the current state is from being stationary.
    float accl_stationary_error = pbio_geometry_absf(pbio_geometry_vector_norm(&acceleration_calibrated) - standard_gravity);
    float gyro_stationary_error = pbio_geometry_absf(pbio_geometry_vector_norm(&angular_velocity_calibrated));

    // Cut off value below which value is considered stationary enough for fusion.
    const float gyro_stationary_min = 10;
    const float accl_stationary_min = 150;

    // Measure for being statinonary ranging from 0 (moving) to 1 (moving less than above thresholds).
    float stationary_measure = accl_stationary_min / pbio_geometry_maxf(accl_stationary_error, accl_stationary_min) *
        gyro_stationary_min / pbio_geometry_maxf(gyro_stationary_error, gyro_stationary_min);

    // The virtual moment would produce motion in that direction, so we can
    // simulate that effect by injecting it into the attitude integration, the
    // strength of which is based on the stationary measure. It is scaled down
    // by the gravity amount since one of the two vectors to produce this has
    // units of gravity. Hence if the hub is stationary (measure = 1), and the
    // error is 90 degrees (which is unlikely), the correction is at
    // most 200 deg/s, but usually much less.
    float fusion = -stationary_measure / standard_gravity * 200;
    pbio_geometry_xyz_t adjusted_angular_velocity;
    adjusted_angular_velocity.x = angular_velocity_calibrated.x + correction.x * fusion;
    adjusted_angular_velocity.y = angular_velocity_calibrated.y + correction.y * fusion;
    adjusted_angular_velocity.z = angular_velocity_calibrated.z + correction.z * fusion;

    // Update 3D attitude, basic forward integration.
    pbio_geometry_quaternion_t dq;
    pbio_geometry_quaternion_get_rate_of_change(&quaternion, &adjusted_angular_velocity, &dq);
    for (uint8_t i = 0; i < PBIO_ARRAY_SIZE(dq.values); i++) {
        quaternion.values[i] += dq.values[i] * periodic_task_interval / 1000000;
    }
    pbio_geometry_quaternion_normalize(&quaternion);

    update_stationary_status(&imu_data);
}


/**
 * Sets the hub base orientation.
 *
 * @param [in]  front_side_axis  Which way the hub front side points when it is
 *                               in the base orientation.
 * @param [in]  top_side_axis    Which way the hub top side points when it is
 *                               in the base orientation.
 * @return                       ::PBIO_SUCCESS on success, ::PBIO_ERROR_INVALID_ARG for incorrect axis values.
 */
pbio_error_t raspike_imu_set_base_orientation(pbio_geometry_xyz_t *front_side_axis, pbio_geometry_xyz_t *top_side_axis) {

    pbio_error_t err = pbio_geometry_map_from_base_axes(front_side_axis, top_side_axis, &pbio_imu_base_orientation);
    if (err != PBIO_SUCCESS) {
        return err;
    }

    // Need to update heading projection since the application axes were changed.
    update_heading_projection();

    // Reset offsets such that the new frame starts with zero heading.
    raspike_imu_set_heading(0.0f);
    return PBIO_SUCCESS;
}

/**
 * Tests if the acceleration value is within a reasonable range for a stationary hub.
 *
 * @param [in]  value  The acceleration value to test.
 * @return             True if the value is within 10% off from standard gravity.
 */
static bool pbio_imu_stationary_acceleration_out_of_range(float value, bool expect_positive) {
    const float expected_value = expect_positive ? standard_gravity : -standard_gravity;
    const float absolute_error = value > expected_value ? value - expected_value : expected_value - value;
    return absolute_error > standard_gravity / 10;
}

/**
 * Tests if a value is close to 360 degrees.
 *
 * @param [in]  value  The value to test.
 * @return             True if the value is within +/-15 degrees of 360, false otherwise.
 */
static bool pbio_imu_setting_close_to_360(float value) {
    return pbio_geometry_absf(value - 360.0f) < 15.0f;
}

/**
 * Initialize the IMU settings with values calibrated using Pybrick's micropython _imu_calibrate class.
 * For IMU calibration process details, refer to https://github.com/pybricks/support/issues/1907
 *
 * @param [in]  gyro_stationary_threshold
 *  The threshold for variations in the angular velocity below which the hub is considered
 *  stationary enough to calibrate. Default value should be 2 deg/s.
 * @param [in]  accel_stationary_threshold
 *  The threshold for variations in acceleration below which the hub is considered
 *  stationary enough to calibrate. Default value should be 2500 mm/s².
 * @param [in]  angular_velocity_bias
 *  Initial bias for angular velocity measurements along x, y, and z immediately after boot.
 *  Default values should be {0, 0, 0} deg/s.
 * @param [in]  angular_velocity_scale
 *  Scale adjustment for x, y, and z rotation to account for manufacturing differences.
 *  Default values should be {360, 360, 360} deg/s
 * @param [in]  acceleration_correction
 *  Scale adjustment for x, y, and z gravity magnitude in both directions to account for manufacturing differences.
 *  Default values should be {9806.65, -9806.65, 9806.65, -9806.65, 9806.65, -9806.65} mm/s².
 * @returns ::PBIO_ERROR_INVALID_ARG if a value is out of range, otherwise ::PBIO_SUCCESS.
 */
pbio_error_t raspike_imu_initialize(float gyro_stationary_threshold, float accel_stationary_threshold,
    float angular_velocity_bias[3], float angular_velocity_scale[3], float acceleration_correction[6]) {

#ifndef TOPPERS_OMIT_SYSLOG
    syslog(LOG_NOTICE, "** Initializing IMU with user configuration values **");
    /* Static buffer ensures they are not destroyed before syslog sends the message to serial port */
    static char gyro_thresh[32], accel_thresh[32];
    static char angv_bias0[32], angv_bias1[32], angv_bias2[32];
    static char angv_scale0[32], angv_scale1[32], angv_scale2[32];
    static char accel_corr0[32], accel_corr1[32], accel_corr2[32], accel_corr3[32], accel_corr4[32], accel_corr5[32];
    FLOAT8_TO_STR(gyro_thresh, gyro_stationary_threshold);
    FLOAT8_TO_STR(accel_thresh, accel_stationary_threshold);
    FLOAT8_TO_STR(angv_bias0, angular_velocity_bias[0]);
    FLOAT8_TO_STR(angv_bias1, angular_velocity_bias[1]);
    FLOAT8_TO_STR(angv_bias2, angular_velocity_bias[2]);
    FLOAT8_TO_STR(angv_scale0, angular_velocity_scale[0]);
    FLOAT8_TO_STR(angv_scale1, angular_velocity_scale[1]);
    FLOAT8_TO_STR(angv_scale2, angular_velocity_scale[2]);
    FLOAT8_TO_STR(accel_corr0, acceleration_correction[0]);
    FLOAT8_TO_STR(accel_corr1, acceleration_correction[1]);
    FLOAT8_TO_STR(accel_corr2, acceleration_correction[2]);
    FLOAT8_TO_STR(accel_corr3, acceleration_correction[3]);
    FLOAT8_TO_STR(accel_corr4, acceleration_correction[4]);
    FLOAT8_TO_STR(accel_corr5, acceleration_correction[5]);
    syslog(LOG_NOTICE, "gyro_stationary_threshold: %s", gyro_thresh);
    syslog(LOG_NOTICE, "accel_stationary_threshold: %s", accel_thresh);
    syslog(LOG_NOTICE, "angular_velocity_bias: %s %s %s",
        angv_bias0, angv_bias1, angv_bias2);
    syslog(LOG_NOTICE, "angular_velocity_scale: %s %s %s",
        angv_scale0, angv_scale1, angv_scale2);
    syslog(LOG_NOTICE, "acceleration_correction: %s %s %s",
        accel_corr0, accel_corr1, accel_corr2);
    syslog(LOG_NOTICE, "                         %s %s %s",
        accel_corr3, accel_corr4, accel_corr5);
#endif /* TOPPERS_OMIT_SYSLOG */

    if (gyro_stationary_threshold < 0 ||
        accel_stationary_threshold < 0) {
        return PBIO_ERROR_INVALID_ARG;
    } else {
        config_gyro_stationary_threshold = gyro_stationary_threshold;
        config_accel_stationary_threshold = accel_stationary_threshold;
    }

    for (uint8_t i = 0; i < 3; i++) {
        gyro_bias.values[i] = angular_velocity_bias[i];

        if (!pbio_imu_setting_close_to_360(angular_velocity_scale[i])) {
            return PBIO_ERROR_INVALID_ARG;
        } else {
            config_angular_velocity_scale.values[i] = angular_velocity_scale[i];
        }

        config_gravity_pos.values[i] = acceleration_correction[2*i];
        config_gravity_neg.values[i] = acceleration_correction[2*i + 1];
        if (pbio_imu_stationary_acceleration_out_of_range(config_gravity_pos.values[i], true) ||
            pbio_imu_stationary_acceleration_out_of_range(config_gravity_neg.values[i], false)) {
            return PBIO_ERROR_INVALID_ARG;
        }
    }

    raspike_imu_initialized = true;
    return PBIO_SUCCESS;
}

/**
 * Initialize the IMU settings with default values that should work for most hubs.
 *
 * @returns ::PBIO_ERROR_INVALID_ARG if a value is out of range, otherwise ::PBIO_SUCCESS.
 */
pbio_error_t raspike_imu_initialize_by_default(void) {
    float default_angular_velocity_bias[3] = {0.0f, 0.0f, 0.0f};
    float default_angular_velocity_scale[3] = {360.0f, 360.0f, 360.0f};
    float default_acceleration_correction[6] = {standard_gravity, -standard_gravity,
        standard_gravity, -standard_gravity, standard_gravity, -standard_gravity};
    return raspike_imu_initialize(2.0f, 2500.0f, default_angular_velocity_bias,
        default_angular_velocity_scale, default_acceleration_correction);
}

/**
 * Initialize the IMU settings with values stored in flash memory.
 *
 * @returns ::PBIO_ERROR_INVALID_ARG if a value is out of range, otherwise ::PBIO_SUCCESS.
 */
pbio_error_t raspike_imu_initialize_by_flash(void) {
    /* to be implemented */
    return raspike_imu_initialize_by_default();
}

/**
 * Gets the cached IMU angular velocity in deg/s, compensated for gyro bias.
 *
 * @param [out] values      The angular velocity vector.
 * @param [in]  calibrated  Whether to get calibrated or uncalibrated data.
 */
void raspike_imu_get_angular_velocity(pbio_geometry_xyz_t *values, bool calibrated) {
    pbio_geometry_xyz_t *angular_velocity = calibrated ? &angular_velocity_calibrated : &angular_velocity_uncalibrated;
    pbio_geometry_vector_map(&pbio_imu_base_orientation, angular_velocity, values);
}

/**
 * Gets the cached IMU acceleration in mm/s^2.
 *
 * @param [in]  calibrated  Whether to use calibrated or uncalibrated data.
 *
 * @param [out] values      The acceleration vector.
 */
void raspike_imu_get_acceleration(pbio_geometry_xyz_t *values, bool calibrated) {
    pbio_geometry_xyz_t *acceleration = calibrated ? &acceleration_calibrated : &acceleration_uncalibrated;
    pbio_geometry_vector_map(&pbio_imu_base_orientation, acceleration, values);
}

/**
 * Gets the tilt-based vector that is parallel to the acceleration measurement.
 *
 * @param [out] values      The acceleration vector.
 */
void raspike_imu_get_tilt_vector(pbio_geometry_xyz_t *values) {
    pbio_geometry_xyz_t direction = {
        .x = pbio_imu_rotation.m31,
        .y = pbio_imu_rotation.m32,
        .z = pbio_imu_rotation.m33,
    };
    pbio_geometry_vector_map(&pbio_imu_base_orientation, &direction, values);
}

/**
 * Gets the rotation along a particular axis of the robot frame.
 *
 * The resulting value makes sense only for one-dimensional rotations.
 *
 * @param [in]  axis        The axis to project the rotation onto.
 * @param [out] angle       The angle of rotation in degrees.
 * @param [in]  calibrated  Whether to use the adjusted scale (true) or the raw scale (false).
 * @return                  ::PBIO_SUCCESS on success, ::PBIO_ERROR_INVALID_ARG if axis has zero length.
 */
pbio_error_t raspike_imu_get_single_axis_rotation(pbio_geometry_xyz_t *axis, float *angle, bool calibrated) {

    // Local copy so we can change it in-place.
    pbio_geometry_xyz_t axis_rotation = single_axis_rotation;
    if (!calibrated) {
        // In this context, calibrated means that the angular velocity values
        // were scaled by the user calibration factors before integrating. This
        // is done within the update loop since we need it for 3D integration.
        // Since this method only returns the separate 1D rotations, we can
        // undo this scaling here to get the "uncalibrated" values.
        for (uint8_t i = 0; i < PBIO_ARRAY_SIZE(axis_rotation.values); i++) {
            axis_rotation.values[i] *= config_angular_velocity_scale.values[i] / 360.0f;
        }
    }

    // Transform the single axis rotations to the robot frame.
    pbio_geometry_xyz_t rotation_user;
    pbio_geometry_vector_map(&pbio_imu_base_orientation, &axis_rotation, &rotation_user);

    // Get the requested scalar rotation along the given axis.
    return pbio_geometry_vector_project(axis, &rotation_user, angle);
}

/**
 * Gets which side of a hub points upwards.
 *
 * @param [in]  calibrated  Whether to use calibrated/fused (true) or raw data (false).
 *
 * @return                  Which side is up.
 */
pbio_geometry_side_t raspike_imu_get_up_side(bool calibrated) {
    // Up is which side of a unit box intersects the +Z vector first.
    // So read +Z vector of the inertial frame, in the body frame.
    // This is either the raw acceleration or the third row of the fused
    // rotation matrix.

    pbio_geometry_xyz_t *vector = &acceleration_uncalibrated;
    if (calibrated) {
        // This is similar to pbio_imu_get_tilt_vector, but this should stay
        // in the hub frame rather than projected into user frame.
        vector->x = pbio_imu_rotation.m31;
        vector->y = pbio_imu_rotation.m32;
        vector->z = pbio_imu_rotation.m33;
    }
    return pbio_geometry_side_from_vector(vector);
}

float heading_offset_1d = 0;
float heading_offset_3d = 0;

/**
 * Reads the estimated IMU heading in degrees, accounting for user offset and
 * user-specified heading correction scaling constant.
 *
 * Heading is defined as clockwise positive.
 *
 * @param [in]  type        The type of heading to get.
 *
 * @return                  Heading angle in the base frame.
 */
float raspike_imu_get_heading(raspike_imu_heading_type_t type) {

    float correction = 1.0f;

    // 3D. Mapping into user frame is already accounted for in the projection.
    if (type == RASPIKE_IMU_HEADING_TYPE_3D) {
        return (heading_rotations * 360.0f + heading_projection) * correction - heading_offset_3d;
    }

    // 1D. Map the per-axis integrated rotation to the user frame, then take
    // the negative z component as the heading for positive-clockwise convention.
    pbio_geometry_xyz_t heading_mapped;
    pbio_geometry_vector_map(&pbio_imu_base_orientation, &single_axis_rotation, &heading_mapped);

    return -heading_mapped.z * correction - heading_offset_1d;
}

/**
 * Sets the IMU heading.
 *
 * This only adjusts the user offset without resetting anything in the
 * algorithm, so this can be called at any time.
 *
 * @param [in] desired_heading  The desired heading value.
 */
void raspike_imu_set_heading(float desired_heading) {
    heading_rotations = 0;
    heading_offset_3d = raspike_imu_get_heading(RASPIKE_IMU_HEADING_TYPE_3D) + heading_offset_3d - desired_heading;
    heading_offset_1d = raspike_imu_get_heading(RASPIKE_IMU_HEADING_TYPE_1D) + heading_offset_1d - desired_heading;
}

/**
 * Gets the estimated IMU heading in control units through a given scale.
 *
 * This is mainly used to convert the heading to the right format for a
 * drivebase, which measures heading as the half the difference of the two
 * motor positions in millidegrees.
 *
 * Heading is defined as clockwise positive.
 *
 * @param [in]   type                  Heading type to get.
 * @param [out]  heading               The heading angle in control units.
 * @param [out]  heading_rate          The heading rate in control units.
 * @param [in]   ctl_steps_per_degree  The number of control steps per heading degree.
 */
void raspike_imu_get_heading_scaled(raspike_imu_heading_type_t type, pbio_angle_t *heading, int32_t *heading_rate, int32_t ctl_steps_per_degree) {

    // Heading in degrees of the robot.
    float heading_degrees = raspike_imu_get_heading(type);

    // Number of whole rotations in control units (in terms of wheels, not robot).
    heading->rotations = (int32_t)(heading_degrees / (360000.0f / ctl_steps_per_degree));

    // The truncated part represents everything else. NB: The scaling factor
    // is a float here to ensure we don't lose precision while scaling.
    float truncated = heading_degrees - heading->rotations * (360000.0f / ctl_steps_per_degree);
    heading->millidegrees = (int32_t)(truncated * ctl_steps_per_degree);

    // The heading rate can be obtained by a simple scale because it always fits.
    pbio_geometry_xyz_t angular_rate;
    raspike_imu_get_angular_velocity(&angular_rate, true);
    *heading_rate = (int32_t)(-angular_rate.z * ctl_steps_per_degree);
}

/**
 * Reads the current rotation matrix.
 *
 * @param [out] rotation      The rotation matrix
 */
void raspike_imu_get_orientation(pbio_geometry_matrix_3x3_t *rotation) {
    *rotation = pbio_imu_rotation;
}


