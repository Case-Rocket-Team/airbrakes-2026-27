#include "control/ekf_adapter.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    extended_kalman_filter ekf{};

    // Simulated EKF output.
    ekf.nominal_state.pos_ft = {100.0f, 200.0f, 3000.0f};
    ekf.nominal_state.vel_fps = {10.0f, 20.0f, 800.0f};

    const auto state = control::vertical_state_from_ekf(ekf);

    // Verify that the adapter extracts the vertical components.
    assert(std::fabs(state.altitude_ft - 3000.0f) < 1e-5f);
    assert(std::fabs(state.vertical_velocity_fps - 800.0f) < 1e-5f);

    // Verify the controller-facing RocketState conversion.
    ekf.state_time_us = 1234567;

    const auto rocket_state = control::rocket_state_from_ekf(ekf);

    assert(std::fabs(rocket_state.altitude_ft - 3000.0f) < 1e-5f);
    assert(std::fabs(rocket_state.vertical_velocity_fps - 800.0f) < 1e-5f);
    assert(rocket_state.timestamp_us == 1234567);

    std::printf("EKF adapter test passed.\n");

    return 0;
}
