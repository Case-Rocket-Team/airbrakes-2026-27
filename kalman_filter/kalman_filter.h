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

#include "../base/base.h"

#define KF_STATE_DIM 3

typedef union {
    struct {
        f32 altitude_ft;
        f32 vertical_vel_fps;
        f32 accel_bias_fps2;
    };

    f32 v[KF_STATE_DIM];
} kf_state;

static_assert(
    sizeof(kf_state) == sizeof(f32) * KF_STATE_DIM,
    "Invalid kalman filter state structure"
);

typedef struct {
    kf_state state;
} kalman_filter;

#ifdef __cplusplus
}
#endif

#endif // KALMAN_FILTER_H
