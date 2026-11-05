#include "control/ekf_adapter.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    ekf_settings settings{};
    settings.accel_var_f2ps4 = 0.01f;
    settings.gyro_var_rad2ps2 = 1e-6f;

    extended_kalman_filter ekf{};
    ekf_init(&ekf, &settings, 0);

    // Known initial vertical state.
    ekf.nominal_state.pos_ft.z = 3000.0f;
    ekf.nominal_state.vel_fps.z = 800.0f;

    // First IMU sample establishes the input.
    ekf_inject_imu(
        &ekf,
        vec3f{0.0f, 0.0f, 0.0f},
        vec3f{0.0f, 0.0f, 0.0f},
        0
    );

    // Advance one second under gravity alone.
    ekf_inject_imu(
        &ekf,
        vec3f{0.0f, 0.0f, 0.0f},
        vec3f{0.0f, 0.0f, 0.0f},
        1000000
    );

    const auto state = control::vertical_state_from_ekf(ekf);

    const float expected_altitude = 3783.913f;
    const float expected_velocity = 767.826f;

    assert(std::fabs(state.altitude_ft - expected_altitude) < 0.1f);
    assert(std::fabs(state.vertical_velocity_fps - expected_velocity) < 0.1f);
    assert(ekf.state_time_us == 1000000);

    std::printf(
        "EKF prediction passed: altitude=%.3f ft, velocity=%.3f ft/s\n",
        state.altitude_ft,
        state.vertical_velocity_fps
    );
    return 0;
}
