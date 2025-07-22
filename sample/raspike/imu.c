// SPDX-License-Identifier: MIT
/**
 * We implement the IMU API by reusing lib/pbio/src/imu.c
 * from the snapshot of pybricks-micropython as of 2024/10/29 @accb117.
 *
 * Original codes Copyright (c) 2023 The Pybricks Authors
 * Modifications for TOPPERS/APS3 Kernel Copyright (c) 2022 Embedded and Real-Time Systems Laboratory,
 *                                                          Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 * The following files have been copied from the same snapshot:
 *   lib/pbio/include/pbio/geometry.h -> sample/raspike/pbio/geometry.h
 *   lib/pbio/src/geometry.c -> sample/raspike/pbio/geometry.c
*/

#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <math.h>

#include <pbio/util.h>

#include <spike/hub/imu.h>
#include "imu.h"
#include "raspike.h"

/**
 * User Configuration Values
 *
 * These values should really be set for each hub by doing a calibration procedure.
 * For now, they are being initialize by default values to make calibrated IMU
 * reading identical to uncalibrated one.
 * For details of IMU calibration, refer to the following post:
 * https://github.com/pybricks/support/issues/1907
 */
/** Positive acceleration values */
pbio_geometry_xyz_t config_gravity_pos;
/** Negative acceleration values */
pbio_geometry_xyz_t config_gravity_neg;
/** Angular velocity scale (unadjusted measured degrees per whole rotation) */
pbio_geometry_xyz_t config_angular_velocity_scale;


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
 * including its bias and adjustments to achhieve 360 degrees per rotation.
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
 * Configures the IMU settings with calibrated values using Pybrick's micropython _imu_calibrate class.
 * For IMU calibration details, refer to https://github.com/pybricks/support/issues/1907
 *
 * @param [in]  angular_velocity_bias
 *  Initial bias for angular velocity measurements along x, y, and z immediately after boot.
 *  Default values are {0, 0, 0} deg/s.
 * @param [in]  ngular_velocity_scale
 *  Scale adjustment for x, y, and z rotation to account for manufacturing differences.
 *  Default values are {360, 360, 360} deg/s
 * @param [in]  acceleration_correction
 *  Scale adjustment for x, y, and z gravity magnitude in both directions to account for manufacturing differences.
 *  Default values are {9806.65, -9806.65, 9806.65, -9806.65, 9806.65, -9806.65} mm/s².
 */
void pbio_imu_set_configuration(const float angular_velocity_bias[3], const float angular_velocity_scale[3], const float acceleration_correction[6]) {
    gyro_bias.x = angular_velocity_bias[0];
    gyro_bias.y = angular_velocity_bias[1];
    gyro_bias.z = angular_velocity_bias[2];
    config_angular_velocity_scale.x = angular_velocity_scale[0];
    config_angular_velocity_scale.y = angular_velocity_scale[1];
    config_angular_velocity_scale.z = angular_velocity_scale[2];
    config_gravity_pos.x = acceleration_correction[0];
    config_gravity_neg.x = acceleration_correction[1];
    config_gravity_pos.y = acceleration_correction[2];
    config_gravity_neg.y = acceleration_correction[3];
    config_gravity_pos.z = acceleration_correction[4];
    config_gravity_neg.z = acceleration_correction[5];
}


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

// Called by periodic handler to process one frame of unfiltered gyro and accelerometer data.
void imu_handle_frame_data_func(void) {
    float angv[3];  // raw angular velocity reading from IMU
    hub_imu_get_angular_velocity(angv);
    float accel[3]; // raw acceleration reading from IMU
    hub_imu_get_acceleration(accel);

    // Initialize quaternion from first gravity sample as a best-effort estimate.
    // From here, fusion will gradually converge the quaternion to the true value.
    if (!quaternion_initialized) {
        pbio_geometry_xyz_t g = { .x = accel[0], .y = accel[1], .z = accel[2]};
        pbio_error_t err = pbio_geometry_vector_normalize(&g, &g);
        if (err != PBIO_SUCCESS) {
            // First sample not suited, try again on next sample.
            return;
        }
        pbio_geometry_quaternion_from_gravity_unit_vector(&g, &quaternion);

        // Initialize angular velocity bias, too.
        // We still update the bias continuously later.
        gyro_bias.x = gyro_bias.y = gyro_bias.z = 0.0f;
        // Initialize User Configuration by default values. 
        config_gravity_pos.x = config_gravity_pos.y = config_gravity_pos.z = standard_gravity;
        config_gravity_neg.x = config_gravity_neg.y = config_gravity_neg.z = -standard_gravity;
        config_angular_velocity_scale.x = config_angular_velocity_scale.y = config_angular_velocity_scale.z = 360.0f; 

        quaternion_initialized = true;
    }

    // Compute current orientation matrix to obtain the current heading.
    pbio_geometry_quaternion_to_rotation_matrix(&quaternion, &pbio_imu_rotation);

    // Projects application x-axis into the inertial frame to compute the heading.
    update_heading_projection();

    for (uint8_t i = 0; i < PBIO_ARRAY_SIZE(angular_velocity_calibrated.values); i++) {
        // Update angular velocity and acceleration cache so user can read them.
        angular_velocity_uncalibrated.values[i] = angv[i];
        acceleration_uncalibrated.values[i] = accel[i];

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
        single_axis_rotation.values[i] += angular_velocity_calibrated.values[i] * PERIOD_GYRO_TSK / 1000000;
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
        quaternion.values[i] += dq.values[i] * PERIOD_GYRO_TSK / 1000000;
    }
    pbio_geometry_quaternion_normalize(&quaternion);
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
pbio_error_t pbio_imu_set_base_orientation(pbio_geometry_xyz_t *front_side_axis, pbio_geometry_xyz_t *top_side_axis) {

    pbio_error_t err = pbio_geometry_map_from_base_axes(front_side_axis, top_side_axis, &pbio_imu_base_orientation);
    if (err != PBIO_SUCCESS) {
        return err;
    }

    // Need to update heading projection since the application axes were changed.
    update_heading_projection();

    // Reset offsets such that the new frame starts with zero heading.
    pbio_imu_set_heading(0.0f);
    return PBIO_SUCCESS;
}

/**
 * Gets the cached IMU angular velocity in deg/s, compensated for gyro bias.
 *
 * @param [out] values      The angular velocity vector.
 * @param [in]  calibrated  Whether to get calibrated or uncalibrated data.
 */
void pbio_imu_get_angular_velocity(pbio_geometry_xyz_t *values, bool calibrated) {
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
void pbio_imu_get_acceleration(pbio_geometry_xyz_t *values, bool calibrated) {
    pbio_geometry_xyz_t *acceleration = calibrated ? &acceleration_calibrated : &acceleration_uncalibrated;
    pbio_geometry_vector_map(&pbio_imu_base_orientation, acceleration, values);
}

/**
 * Gets the vector that is parallel to the acceleration measurement of the stationary case.
 *
 * @param [out] values      The acceleration vector.
 */
void pbio_imu_get_tilt_vector(pbio_geometry_xyz_t *values) {
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
 * @return                  ::PBIO_SUCCESS on success, ::PBIO_ERROR_INVALID_ARG if axis has zero length.
 */
pbio_error_t pbio_imu_get_single_axis_rotation(pbio_geometry_xyz_t *axis, float *angle) {

    // Transform the single axis rotations to the robot frame.
    pbio_geometry_xyz_t rotation;
    pbio_geometry_vector_map(&pbio_imu_base_orientation, &single_axis_rotation, &rotation);

    // Get the requested scalar rotation along the given axis.
    return pbio_geometry_vector_project(axis, &rotation, angle);
}

/**
 * Gets which side of a hub points upwards.
 *
 * @param [in]  calibrated  Whether to use calibrated or uncalibrated data.
 *
 * @return                  Which side is up.
 */
pbio_geometry_side_t pbio_imu_get_up_side(bool calibrated) {
    // Up is which side of a unit box intersects the +Z vector first.
    // So read +Z vector of the inertial frame, in the body frame.
    // For now, this is the gravity vector. In the future, we can make this
    // slightly more accurate by using the full IMU orientation.
    pbio_geometry_xyz_t *acceleration = calibrated ? &acceleration_calibrated : &acceleration_uncalibrated;
    return pbio_geometry_side_from_vector(acceleration);
}

static float heading_offset = 0;

/**
 * Reads the estimated IMU heading in degrees, accounting for user offset and
 * user-specified heading correction scaling constant.
 *
 * Heading is defined as clockwise positive.
 *
 * @return                  Heading angle in the base frame.
 */
float pbio_imu_get_heading(void) {
    return heading_rotations * 360.0f + heading_projection - heading_offset;
}

/**
 * Sets the IMU heading.
 *
 * This only adjusts the user offset without resetting anything in the
 * algorithm, so this can be called at any time.
 *
 * @param [in] desired_heading  The desired heading value.
 */
void pbio_imu_set_heading(float desired_heading) {
    heading_rotations = 0;
    heading_offset = pbio_imu_get_heading() + heading_offset - desired_heading;
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
 * @param [out]  heading               The heading angle in control units.
 * @param [out]  heading_rate          The heading rate in control units.
 * @param [in]   ctl_steps_per_degree  The number of control steps per heading degree.
 */
void pbio_imu_get_heading_scaled(pbio_angle_t *heading, int32_t *heading_rate, int32_t ctl_steps_per_degree) {

    // Heading in degrees of the robot.
    float heading_degrees = pbio_imu_get_heading();

    // Number of whole rotations in control units (in terms of wheels, not robot).
    heading->rotations = (int32_t)(heading_degrees / (360000.0f / ctl_steps_per_degree));

    // The truncated part represents everything else. NB: The scaling factor
    // is a float here to ensure we don't lose precision while scaling.
    float truncated = heading_degrees - heading->rotations * (360000.0f / ctl_steps_per_degree);
    heading->millidegrees = (int32_t)(truncated * ctl_steps_per_degree);

    // The heading rate can be obtained by a simple scale because it always fits.
    pbio_geometry_xyz_t angular_rate;
    pbio_imu_get_angular_velocity(&angular_rate, true);
    *heading_rate = (int32_t)(-angular_rate.z * ctl_steps_per_degree);
}

/**
 * Reads the current rotation matrix.
 *
 * @param [out] rotation      The rotation matrix
 */
void pbio_orientation_imu_get_rotation(pbio_geometry_matrix_3x3_t *rotation) {
    *rotation = pbio_imu_rotation;
}


