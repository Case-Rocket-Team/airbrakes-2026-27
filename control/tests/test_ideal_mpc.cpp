#include "../controller/ideal_mpc.hpp"
#include "../predictor/apogee_predictor.hpp"

#include <cassert>
#include <cmath>

int main() {
    const control::physics::VerticalState state{
        10000.0f,
        500.0f
    };

    const control::physics::RocketParameters rocket_parameters{
        2.0f,
        0.5f
    };

    const control::aero::AeroParameters aero_parameters{
        0.30f,
        0.20f
    };

    constexpr f32 prediction_dt = 0.01f;

    const f32 retracted_apogee = control::predictor::predict_apogee(
        state,
        rocket_parameters,
        aero_parameters,
        0.0f,
        prediction_dt
    );

    const f32 half_apogee = control::predictor::predict_apogee(
        state,
        rocket_parameters,
        aero_parameters,
        0.5f,
        prediction_dt
    );

    const f32 full_apogee = control::predictor::predict_apogee(
        state,
        rocket_parameters,
        aero_parameters,
        1.0f,
        prediction_dt
    );

    // A target above the reachable range should retract the brakes.
    {
        const control::MpcParameters mpc_parameters{
            retracted_apogee + 1000.0f,
            prediction_dt,
            0.25f
        };

        const control::BrakeCommand command = control::ideal_mpc(
            state,
            rocket_parameters,
            aero_parameters,
            mpc_parameters
        );

        assert(std::fabs(command.extension - 0.0f) < 1e-5f);
    }

    // A target matching an intermediate candidate should select it.
    {
        const control::MpcParameters mpc_parameters{
            half_apogee,
            prediction_dt,
            0.25f
        };

        const control::BrakeCommand command = control::ideal_mpc(
            state,
            rocket_parameters,
            aero_parameters,
            mpc_parameters
        );

        assert(std::fabs(command.extension - 0.5f) < 1e-5f);
    }

    // A target below the reachable range should fully extend the brakes.
    // The 0.3 step also verifies that the 1.0 endpoint is always evaluated.
    {
        const control::MpcParameters mpc_parameters{
            full_apogee - 1000.0f,
            prediction_dt,
            0.3f
        };

        const control::BrakeCommand command = control::ideal_mpc(
            state,
            rocket_parameters,
            aero_parameters,
            mpc_parameters
        );

        assert(std::fabs(command.extension - 1.0f) < 1e-5f);
    }

    // Invalid candidate resolution should fail safely with retracted brakes.
    {
        const control::MpcParameters mpc_parameters{
            half_apogee,
            prediction_dt,
            0.0f
        };

        const control::BrakeCommand command = control::ideal_mpc(
            state,
            rocket_parameters,
            aero_parameters,
            mpc_parameters
        );

        assert(std::fabs(command.extension - 0.0f) < 1e-5f);
    }

    return 0;
}