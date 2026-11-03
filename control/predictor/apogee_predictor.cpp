#include "control/predictor/apogee_predictor.hpp"
#include "control/aero/brake_aero.hpp"

#include <cmath>

namespace control::predictor {

f32 predict_apogee(
    const physics::VerticalState& initial_state,
    const physics::RocketParameters& parameters,
    f32 drag_coefficient,
    f32 dt
) {
    physics::VerticalState state = initial_state;

    while (state.vertical_velocity_fps > 0.0f) {
        state = physics::propagate_vertical(
            state,
            parameters,
            drag_coefficient,
            dt
        );
    }

    return state.altitude_ft;
}

f32 predict_apogee(
    const physics::VerticalState& initial_state,
    const physics::RocketParameters& parameters,
    const aero::AeroParameters& aero_parameters,
    f32 brake_extension,
    f32 dt
) {
    const f32 drag_coefficient =
        aero::drag_coefficient(
            aero_parameters,
            brake_extension
        );

    return predict_apogee(
        initial_state,
        parameters,
        drag_coefficient,
        dt
    );
}


f32 predict_apogee(
    const physics::VerticalState& initial_state,
    const physics::RocketParameters& parameters,
    const aero::AeroParameters& aero_parameters,
    f32 current_extension,
    f32 commanded_extension,
    f32 dt,
    const actuator::ActuatorParameters& actuator_parameters
) {
    physics::VerticalState state = initial_state;
    f32 extension = current_extension;

    // Reject non-finite flight states.
    if (!std::isfinite(state.altitude_ft) ||
        !std::isfinite(state.vertical_velocity_fps)) {
        return NAN;
    }

    // Invalid timestep: prediction cannot be trusted.
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return NAN;
    }

    // Prevent an unbounded prediction loop.
    constexpr int MAX_PREDICTION_STEPS = 100000;
    int prediction_steps = 0;

    while (state.vertical_velocity_fps > 0.0f &&
        prediction_steps < MAX_PREDICTION_STEPS) {
        ++prediction_steps;

        // Update actuator position based on slew rate.
        const f32 next_position = actuator::next_extension(
            extension,
            commanded_extension,
            dt,
            actuator_parameters
        );

        // Approximate average extension during this timestep.
        const f32 average_extension =
            0.5f * (extension + next_position);

        // Calculate drag at the average extension.
        const f32 cd = aero::drag_coefficient(
            aero_parameters,
            average_extension
        );

        // Propagate rocket dynamics using existing physics.
        const physics::VerticalState previous_state = state;

        state = physics::propagate_vertical(
            state,
            parameters,
            cd,
            dt
        );

        // Reject invalid physics results.
        if (!std::isfinite(state.altitude_ft) ||
            !std::isfinite(state.vertical_velocity_fps)) {
            return NAN;
        }

        if (state.vertical_velocity_fps <= 0.0f) {
            const f32 fraction =
                previous_state.vertical_velocity_fps /
                (previous_state.vertical_velocity_fps - state.vertical_velocity_fps);

            return previous_state.altitude_ft +
                previous_state.vertical_velocity_fps * dt * fraction +
                0.5f *
                ((state.vertical_velocity_fps - previous_state.vertical_velocity_fps) / dt) *
                (dt * fraction) * (dt * fraction);
        }

            extension = next_position;
        }

        // Prediction stopped before apogee: report failure.
        if (state.vertical_velocity_fps > 0.0f) {
            return NAN;
        }

        return state.altitude_ft;
    }

}  // namespace control::predictor
