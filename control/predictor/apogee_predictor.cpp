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

    // Invalid timestep: return current altitude.
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return state.altitude_ft;
    }

    while (state.vertical_velocity_fps > 0.0f) {

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
        state = physics::propagate_vertical(
            state,
            parameters,
            cd,
            dt
        );

        extension = next_position;
    }

    return state.altitude_ft;
}

}  // namespace control::predictor
