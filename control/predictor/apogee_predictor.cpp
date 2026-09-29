#include "control/predictor/apogee_predictor.hpp"
#include "control/aero/brake_aero.hpp"

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

}  // namespace control::predictor
