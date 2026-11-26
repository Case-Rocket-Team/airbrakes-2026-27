//
// Created by jcg on 10/13/26.
//

#ifndef BINDINGS_TEST_H
#define BINDINGS_TEST_H
#include <tuple>

#include "kalman_filter/extended_kalman_filter.h"

class ExtendedKalmanFilter {
private:
        extended_kalman_filter* EKF;
public:
    ExtendedKalmanFilter(
        const ekf_settings* settings,
        vec3f local_down,
        vec3f local_magn_gauss,
        u32 timestamp_us);
    ~ExtendedKalmanFilter();

    // Call with IMU data
    void inject_imu(
        vec3f accel_fps2,
        vec3f gyro_radps,
        u32 timestamp_us);

    // Call with new barometeric altitude data
    void inject_baro(
        f32 altitude_ft,
        u32 timestamp_us);

    // Call with new magnetometer data
    void inject_magn(
        vec3f magn_north_gauss,
        u32 timestamp_us);

    // Call with new GPS data
    void inject_gnss(
        vec3f pos_ft,
        u32 timestamp_us);
    ekf_nominal_state get_nominal_state();

    f32* get_state_covar();

};

#endif //BINDINGS_TEST_H