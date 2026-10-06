#include "ideal_mpc.hpp"

#include "../predictor/apogee_predictor.hpp"

#include <cmath>

namespace control {

BrakeCommand ideal_mpc(
    const physics::VerticalState& state,
    const physics::RocketParameters& rocket_parameters,
    const aero::AeroParameters& aero_parameters,
    const MpcParameters& mpc_parameters
) {

    if (mpc_parameters.extension_step <= 0.0f) {
        return BrakeCommand{0.0f};
    }

    f32 best_extension = 0.0f;
    f32 best_error = INFINITY;

    for (
        f32 extension = 0.0f;
        extension <= 1.0f;
        extension += mpc_parameters.extension_step
    ) {
        const f32 predicted_apogee = predictor::predict_apogee(
            state,
            rocket_parameters,
            aero_parameters,
            extension,
            mpc_parameters.prediction_dt
        );

        const f32 error =
            std::fabs(predicted_apogee - mpc_parameters.target_apogee_ft);

        if (error < best_error) {
            best_error = error;
            best_extension = extension;
        }
    }

    // Always evaluate full extension in case extension_step does not
    // divide the [0, 1] range evenly.
    const f32 full_extension_apogee = predictor::predict_apogee(
        state,
        rocket_parameters,
        aero_parameters,
        1.0f,
        mpc_parameters.prediction_dt
    );

    const f32 full_extension_error =
        std::fabs(full_extension_apogee - mpc_parameters.target_apogee_ft);

    if (full_extension_error < best_error) {
        best_extension = 1.0f;
    }

    return BrakeCommand{best_extension};
}

} // namespace control
