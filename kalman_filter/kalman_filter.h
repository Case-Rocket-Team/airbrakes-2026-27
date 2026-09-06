/*

File kalman_filter.h
-----------------------

This is a simple linear kalman filter that I am using to learn about writing
kalman filters. Any actual flight code needs to use a filter that incorporates
rotation.

*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "base/base.h"

#define KF_STATE_DIM 3
#define KF_CONTROL_DIM 1
#define KF_MEASURE_DIM 1

typedef struct {
    union {
        struct {
            f32 altitude_ft;
            f32 vertical_vel_fps;
            f32 accel_bias_fps2;
        };

        f32 v[KF_STATE_DIM];
    } vec;

    f32 covariance[KF_STATE_DIM * KF_STATE_DIM];
} kf_state;

typedef struct {
    union {
        struct {
            f32 accel_fps2;
        };

        f32 v[KF_CONTROL_DIM];
    } vec;

    f32 covariance[KF_CONTROL_DIM * KF_CONTROL_DIM];
} kf_control_input;

typedef struct {
    union {
        struct {
            f32 altitude_ft;
        };

        f32 v[KF_MEASURE_DIM];
    } vec;

    f32 covariance[KF_MEASURE_DIM * KF_MEASURE_DIM];
} kf_measure;

typedef struct {
    kf_state state;
} kalman_filter;

void kf_predict(kalman_filter* kf, kf_control_input* control, f32 dt);
void kf_update(kalman_filter* kf, kf_measure* measure, f32 dt);

#ifdef __cplusplus
}
#endif

#endif // KALMAN_FILTER_H
