// SPDX-License-Identifier: MIT

/**
 * We define the IMU API by reusing lib/pbio/include/pbio/imu.h
 * from the snapshot of pybricks-micropython as of 2025/02/25 @57793e3.
 *
 * Original codes Copyright (c) 2022-2023 The Pybricks Authors
 * Modifications for TOPPERS/APS3 Kernel Copyright (c) 2022 Embedded and Real-Time Systems Laboratory,
 *                                                          Graduate School of Information Science, Nagoya Univ., JAPAN
 */

#ifndef __RASPIKE_IMU_H__
#define __RASPIKE_IMU_H__

#include <stdint.h>

#include <pbio/angle.h>
#include <pbio/error.h>
#include <pbio/geometry.h>

/**
 * Heading type to use, set, or get.
 */
typedef enum {
    /**
     * Heading should not be used.
     */
    RASPIKE_IMU_HEADING_TYPE_NONE,
    /**
     * The heading is the integrated gyro rate along one fixed axis.
     */
    RASPIKE_IMU_HEADING_TYPE_1D,
    /**
     * The heading is angle between the projection of the line coming out of
     * the front of the hub onto the horizontal plane and the x-axis.
     */
    RASPIKE_IMU_HEADING_TYPE_3D,
} raspike_imu_heading_type_t;

bool raspike_imu_is_ready(void);

void raspike_imu_handle_frame_data(uint32_t periodic_task_interval);

pbio_error_t raspike_imu_set_base_orientation(pbio_geometry_xyz_t *x_axis, pbio_geometry_xyz_t *z_axis);

pbio_error_t raspike_imu_initialize(float gyro_stationary_threshold, float accel_stationary_threshold,
    float angular_velocity_bias[3], float angular_velocity_scale[3], float acceleration_correction[6]);

pbio_error_t raspike_imu_initialize_by_default(void);

pbio_error_t raspike_imu_initialize_by_flash(void);

void raspike_imu_get_angular_velocity(pbio_geometry_xyz_t *values, bool calibrated);

void raspike_imu_get_acceleration(pbio_geometry_xyz_t *values, bool calibrated);

void raspike_imu_get_tilt_vector(pbio_geometry_xyz_t *values);

pbio_error_t raspike_imu_get_single_axis_rotation(pbio_geometry_xyz_t *axis, float *angle, bool calibrated);

pbio_geometry_side_t raspike_imu_get_up_side(bool calibrated);

float raspike_imu_get_heading(raspike_imu_heading_type_t type);

void raspike_imu_set_heading(float desired_heading);

void raspike_imu_get_heading_scaled(raspike_imu_heading_type_t type, pbio_angle_t *heading, int32_t *heading_rate, int32_t ctl_steps_per_degree);

void raspike_imu_get_orientation(pbio_geometry_matrix_3x3_t *rotation);

/* -------------------------------------------------------------
 *  FLOAT8_TO_STR(buf, value)
 *
 *  Formats a floating‑point number into the character array `buf`
 *  (must be large enough).  The output has exactly eight digits
 *  after the decimal point and is NUL terminated.
 *
 *  No stdio functions are used – only integer arithmetic and
 *  plain assignments, so it works on systems where printf‑style
 *  zero‑padding is unavailable.
 *
 *  Parameters
 *      buf   – a char array (e.g. char txt[32];).  The macro will
 *              write the result into it and always terminate with '\0'.
 *
 *      value – the floating point number to convert (float, double,
 *              long double …).  It is promoted to `double` inside.
 *
 *  Example
 *      char s[32];
 *      FLOAT8_TO_STR(s, -3.1415926535);
 *          // s now contains "-3.14159265"
 * ------------------------------------------------------------- */
#define FLOAT8_TO_STR(buf, value)                                         \
    do {                                                                   \
        /* ---------- 1. sign handling -------------------------------- */\
        int _f8_sign = ((value) < 0.0) ? -1 : 1;                           \
        double _f8_abs = (double)(value) * _f8_sign;                      \
                                                                             \
        /* ---------- 2. split integer / fraction --------------------- */\
        long long _int_part = (long long)_f8_abs;                          \
        double   _frac_d   = _f8_abs - (double)_int_part;                 \
                                                                             \
        /* ---------- 3. scale fraction to 10⁸ and round ------------- */\
        const unsigned _PREC = 8U;                                        \
        long long _frac_int = (long long)(_frac_d * 100000000.0 + 0.5);   \
                                                                             \
        /* ---------- 4. prepare buffer pointer ----------------------- */\
        char *_p = (buf);                                                 \
                                                                             \
        /* ---------- 5. write sign if needed -------------------------- */\
        if (_f8_sign < 0) {                                                \
            *_p++ = '-';                                                   \
        }                                                                  \
                                                                             \
        /* ---------- 6. convert integer part to decimal string -------- */\
        /*   We write the digits into a temporary buffer in reverse order */\
        char _tmp[32];                                                     \
        int  _len = 0;                                                    \
        if (_int_part == 0) {                                             \
            _tmp[_len++] = '0';                                            \
        } else {                                                          \
            long long _v = _int_part;                                      \
            while (_v != 0) {                                              \
                int digit = (int)(_v % 10LL);                              \
                _tmp[_len++] = (char)('0' + digit);                       \
                _v /= 10LL;                                                \
            }                                                             \
        }                                                                 \
        /* now copy the digits back in correct order */                    \
        while (_len--) {                                                   \
            *_p++ = _tmp[_len];                                            \
        }                                                                  \
                                                                             \
        /* ---------- 7. decimal point --------------------------------- */\
        *_p++ = '.';                                                       \
                                                                             \
        /* ---------- 8. write exactly eight fraction digits ----------- */\
        /*   The fraction is already an integer in [_frac_int] (0 … 99 999 999) */\
        unsigned _div = 10000000U;      /* 10⁷, the highest divisor */     \
        for (unsigned i = 0; i < _PREC; ++i) {                             \
            int digit = (int)(_frac_int / _div);                           \
            *_p++ = (char)('0' + digit);                                   \
            _frac_int %= _div;                                             \
            _div /= 10U;                                                   \
        }                                                                  \
                                                                             \
        /* ---------- 9. NUL‑terminate --------------------------------- */\
        *_p = '\0';                                                        \
    } while (0)

#endif // __RASPIKE_IMU_H__

/** @} */
