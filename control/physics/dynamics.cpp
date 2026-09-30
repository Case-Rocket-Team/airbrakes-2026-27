#include "control/physics/dynamics.hpp"

#include "control/physics/atmosphere.hpp"
#include "control/physics/drag.hpp"

namespace control::physics {

namespace {

constexpr f32 GRAVITY_FPS2 = 32.174f;

}  // namespace

VerticalState propagate_vertical(
    const VerticalState& state,
    const RocketParameters& parameters,
    f32 drag_coefficient,
    f32 dt
) {
    const f32 density = air_density(state.altitude_ft);

    const f32 drag = drag_force(
        density,
        state.vertical_velocity_fps,
        drag_coefficient,
        parameters.reference_area_ft2
    );

    const f32 drag_acceleration = drag / parameters.mass_slug;

    const f32 acceleration =
        -GRAVITY_FPS2 - drag_acceleration;

    VerticalState next_state;

    next_state.altitude_ft =
        state.altitude_ft +
        state.vertical_velocity_fps * dt +
        0.5f * acceleration * dt * dt;

    next_state.vertical_velocity_fps =
        state.vertical_velocity_fps +
        acceleration * dt;

    return next_state;
}

}  // namespace control::physics