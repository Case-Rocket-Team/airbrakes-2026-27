/*

File extended_kalman_filter.h
-----------------------

This is an error state extended kalman filter that tracks rocket's attitude as
a quaternion. It also tracks position, velocity, and accelerometer and 
gyroscope bias.

*/

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "base/base.h"

#define EKF_STATE_DIM 18
#define EKF_CONTROL_DIM 6
#define EKF_MEASURE_DIM 4

typedef struct {
    // Stored such that left multiplication by attitude goes from body frame
    // to world frame
    quatf attitude;

    // Position is in the world's frame of reference
    vec3f pos_ft;

    // Velocity is in the world's frame of reference
    vec3f vel_fps;

    vec3f gyro_bias_radps; // radians / sec
    vec3f accel_bias_fps2;
    vec3f magn_bias_gauss;
} ekf_nominal_state;

typedef union {
    struct {
        vec3f small_angle_rad;
        vec3f vel_fps;
        vec3f pos_ft;
        vec3f gyro_bias_radps;
        vec3f accel_bias_fps2;
        vec3f magn_bias_gauss;
    };
    
    f32 v[EKF_STATE_DIM];
} ekf_err_state;

static_assert(sizeof(ekf_err_state) == sizeof(f32) * EKF_STATE_DIM);

typedef union {
    struct {
        vec3f accel_fps2;
        vec3f gyro_radps;
    };

    f32 v[EKF_CONTROL_DIM];
} ekf_control_input;

static_assert(sizeof(ekf_control_input) == sizeof(f32) * EKF_CONTROL_DIM);

typedef union {
    struct {
        // From barometer
        f32 altitude_ft;

        // From magnetometer 
        vec3f magn_north_gauss;
    };

    f32 v[EKF_MEASURE_DIM];
} ekf_measure;

static_assert(sizeof(ekf_measure) == sizeof(f32) * EKF_MEASURE_DIM);

typedef struct {
    ekf_nominal_state nominal_state;
    ekf_err_state err_state;

    f32 state_cov[EKF_STATE_DIM * EKF_STATE_DIM];

    // Should be normalized
    vec3f world_magn_north;

    f32 accel_var_f2ps4;
    f32 accel_bias_var_f2ps4;
    f32 gyro_var_rad2ps2;
    f32 gyro_bias_var_rad2ps2;
    f32 magn_bias_var_gauss2;

    f32 measure_covar[EKF_MEASURE_DIM * EKF_MEASURE_DIM];
} extended_kalman_filter;

/*
Predicts the state at dt seconds into the future.

Should be called before ekf_update
*/
void ekf_predict(
    extended_kalman_filter* ekf, ekf_control_input* control, f32 dt
);

/*
Should be proceeded by an ekf_predict such that the time at the ekf_update call
is equal to the time at the latest kf_predict + dt
*/
void ekf_update(extended_kalman_filter* ekf, ekf_measure* measure);

#ifdef __cplusplus
}
#endif

#endif // EXTENDED_KALMAN_FILTER_H
