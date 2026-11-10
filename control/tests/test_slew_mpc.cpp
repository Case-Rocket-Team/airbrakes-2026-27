
#include "control/controller/slew_mpc.hpp"
#include "control/predictor/apogee_predictor.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    using namespace control;

    const physics::VerticalState state{5000.0f, 500.0f};
    const physics::RocketParameters rocket{1.0f, 0.1f};
    const aero::AeroParameters aero{0.4f, 0.6f};
    const MpcParameters mpc{8000.0f, 0.01f, 0.1f};
    const actuator::ActuatorParameters actuator{0.2f};

    // Test 1: Command stays within valid extension bounds.
    const BrakeCommand command = slew_mpc(
        state, rocket, aero, mpc, 0.0f, actuator
    );

    assert(std::isfinite(command.extension));
    assert(command.extension >= 0.0f);
    assert(command.extension <= 1.0f);

    // Test 2: Invalid candidate step returns retracted command.
    const MpcParameters invalid_step{8000.0f, 0.01f, 0.0f};

    const BrakeCommand invalid_command = slew_mpc(
        state, rocket, aero, invalid_step, 0.0f, actuator
    );

    assert(invalid_command.extension == 0.0f);

    // Test 3: Selected command is no worse than the endpoints.
    const f32 selected_apogee = predictor::predict_apogee(
        state, rocket, aero, 0.0f, command.extension,
        mpc.prediction_dt, actuator
    );

    const f32 retracted_apogee = predictor::predict_apogee(
        state, rocket, aero, 0.0f, 0.0f,
        mpc.prediction_dt, actuator
    );

    const f32 extended_apogee = predictor::predict_apogee(
        state, rocket, aero, 0.0f, 1.0f,
        mpc.prediction_dt, actuator
    );

    const f32 selected_error =
        std::fabs(selected_apogee - mpc.target_apogee_ft);

    const f32 retracted_error =
        std::fabs(retracted_apogee - mpc.target_apogee_ft);

    const f32 extended_error =
        std::fabs(extended_apogee - mpc.target_apogee_ft);

    assert(selected_error <= retracted_error + 0.01f);
    assert(selected_error <= extended_error + 0.01f);

    std::cout << "All slew MPC tests passed.\n";
    return 0;
}
