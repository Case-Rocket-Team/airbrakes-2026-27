#include "control/predictor/apogee_predictor.hpp"

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

}  // namespace control::predictor