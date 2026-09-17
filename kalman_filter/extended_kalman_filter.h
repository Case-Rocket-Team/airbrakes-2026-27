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

#define EKF_STATE_DIM 15
#define EKF_CONTROL_DIM 6
#define EKF_MEASURE_DIM 4

typedef struct {
    quatf attitude;
    vec3f pos_ft;
    vec3f vel_fps;
    vec3f accel_bias_fps2;
    vec3f gyro_bias_radps; // radians / sec
} ekf_nominal_state;

typedef struct {
    union {
        struct {
            vec3f small_angle_rad;
            vec3f pos_ft;
            vec3f vel_fps;
            vec3f accel_bias_fps2;
            vec3f gyro_bias_radps;
        };
        
        f32 v[EKF_STATE_DIM];
    } vec;

    f32 covar[EKF_STATE_DIM * EKF_STATE_DIM];
} ekf_err_state;

typedef union {
    struct {
        vec3f accel_fps2;
        vec3f gyro_radps;
    };

    f32 v[EKF_CONTROL_DIM];
} ekf_control_input;

typedef union {
    struct {
        // From barometer
        f32 altitude_ft;

        // From magnetometer (units do not particularly matter for this one)
        vec3f magn_north_gauss;
    };

    f32 v[EKF_MEASURE_DIM];
} ekf_measure;

typedef struct {
    ekf_nominal_state nominal_state;
    ekf_err_state err_state;

    f32 accel_var_f2ps4;
    f32 accel_bias_var_f2ps4;
    f32 gyro_var_rad2ps2;
    f32 gyro_bias_var_rad2ps2;

    f32 measure_covar[EKF_MEASURE_DIM * EKF_MEASURE_DIM];
} extended_kalman_filter;

/*
Predicts the state at dt seconds into the future.

Should be called before ekf_update
*/
void ekf_predict(extended_kalman_filter* ekf, ekf_control_input* control, f32 dt);

/*
Should be proceeded by an ekf_predict such that the time at the ekf_update call
is equal to the time at the latest kf_predict + dt
*/
void ekf_update(extended_kalman_filter* ekf, ekf_measure* measure);

#ifdef __cplusplus
}
#endif

#endif // EXTENDED_KALMAN_FILTER_H
