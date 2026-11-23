
#include "control/controller/slew_mpc.hpp"
#include "control/predictor/apogee_predictor.hpp"

#include <cmath>

namespace control {

BrakeCommand slew_mpc(
    const physics::VerticalState& state,
    const physics::RocketParameters& rocket_parameters,
    const aero::AeroParameters& aero_parameters,
    const MpcParameters& mpc_parameters,
    f32 current_extension,
    const actuator::ActuatorParameters& actuator_parameters
) {
    // Reject invalid controller configuration.
    if (!std::isfinite(mpc_parameters.extension_step) ||
        mpc_parameters.extension_step <= 0.0f ||
        !std::isfinite(mpc_parameters.prediction_dt) ||
        mpc_parameters.prediction_dt <= 0.0f ||
        !std::isfinite(mpc_parameters.target_apogee_ft)) {
        return BrakeCommand{0.0f};
    }

    f32 best_extension = 0.0f;
    f32 best_error = INFINITY;

    // Evaluate candidate commanded brake extensions.
    for (
        f32 extension = 0.0f;
        extension <= 1.0f;
        extension += mpc_parameters.extension_step
    ) {
        const f32 predicted_apogee = predictor::predict_apogee(
            state,
            rocket_parameters,
            aero_parameters,
            current_extension,
            extension,
            mpc_parameters.prediction_dt,
            actuator_parameters
        );

        const f32 error = std::fabs(
            predicted_apogee - mpc_parameters.target_apogee_ft
        );

        if (error < best_error) {
            best_error = error;
            best_extension = extension;
        }
    }

    // Explicitly evaluate 100% extension.
    const f32 full_extension_apogee = predictor::predict_apogee(
        state,
        rocket_parameters,
        aero_parameters,
        current_extension,
        1.0f,
        mpc_parameters.prediction_dt,
        actuator_parameters
    );

    const f32 full_extension_error = std::fabs(
        full_extension_apogee - mpc_parameters.target_apogee_ft
    );

    if (full_extension_error < best_error) {
        best_extension = 1.0f;
    }

    return BrakeCommand{best_extension};
}

} // namespace control
