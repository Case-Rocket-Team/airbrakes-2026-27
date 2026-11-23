#include "control/ekf_adapter.hpp"
#include "control/controller/slew_mpc.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    using namespace control;

    // Synthetic EKF output representing a rocket during coast.
    extended_kalman_filter ekf{};
    ekf.nominal_state.pos_ft = {0.0f, 0.0f, 5000.0f};
    ekf.nominal_state.vel_fps = {0.0f, 0.0f, 500.0f};
    ekf.state_time_us = 1000000;

    // Existing controller test parameters.
    const physics::RocketParameters rocket{1.0f, 0.1f};
    const aero::AeroParameters aero{0.4f, 0.6f};
    const MpcParameters mpc{8000.0f, 0.01f, 0.1f};
    const actuator::ActuatorParameters actuator{0.2f};

    // Controller input obtained through the EKF adapter.
    const physics::VerticalState estimated_state =
        vertical_state_from_ekf(ekf);

    const BrakeCommand estimated_command = slew_mpc(
        estimated_state, rocket, aero, mpc, 0.0f, actuator
    );

    // Equivalent controller input supplied directly.
    const physics::VerticalState direct_state{5000.0f, 500.0f};

    const BrakeCommand direct_command = slew_mpc(
        direct_state, rocket, aero, mpc, 0.0f, actuator
    );

    // Both paths should produce identical commands.
    assert(std::isfinite(estimated_command.extension));
    assert(estimated_command.extension >= 0.0f);
    assert(estimated_command.extension <= 1.0f);

    assert(
        std::fabs(estimated_command.extension -
                  direct_command.extension) < 1e-6f
    );

    std::printf("EKF-to-MPC integration test passed.\n");
    return 0;
}
