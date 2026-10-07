#include "control/physics/dynamics.hpp"

#include "control/physics/atmosphere.hpp"
#include "control/physics/drag.hpp"

namespace control::physics {

namespace {

constexpr f32 GRAVITY_FPS2 = 32.174f;

f32 vertical_acceleration(
    const VerticalState& state,
    const RocketParameters& parameters,
    f32 drag_coefficient
) {
    const f32 density = air_density(state.altitude_ft);

    const f32 drag = drag_force(
        density,
        state.vertical_velocity_fps,
        drag_coefficient,
        parameters.reference_area_ft2
    );

    return -GRAVITY_FPS2 - drag / parameters.mass_slug;
}

}  // namespace

VerticalState propagate_vertical(
    const VerticalState& state,
    const RocketParameters& parameters,
    f32 drag_coefficient,
    f32 dt
) {
    const f32 k1_altitude = state.vertical_velocity_fps;
    const f32 k1_velocity = vertical_acceleration(state, parameters, drag_coefficient);

    const VerticalState state_k2{
        state.altitude_ft + 0.5f * dt * k1_altitude,
        state.vertical_velocity_fps + 0.5f * dt * k1_velocity
    };

    const f32 k2_altitude = state_k2.vertical_velocity_fps;
    const f32 k2_velocity = vertical_acceleration(state_k2, parameters, drag_coefficient);

    const VerticalState state_k3{
        state.altitude_ft + 0.5f * dt * k2_altitude,
        state.vertical_velocity_fps + 0.5f * dt * k2_velocity
    };

    const f32 k3_altitude = state_k3.vertical_velocity_fps;
    const f32 k3_velocity = vertical_acceleration(state_k3, parameters, drag_coefficient);

    const VerticalState state_k4{
        state.altitude_ft + dt * k3_altitude,
        state.vertical_velocity_fps + dt * k3_velocity
    };

    const f32 k4_altitude = state_k4.vertical_velocity_fps;
    const f32 k4_velocity = vertical_acceleration(state_k4, parameters, drag_coefficient);

    VerticalState next_state;

    next_state.altitude_ft =
        state.altitude_ft +
        dt / 6.0f *
        (k1_altitude + 2.0f * k2_altitude +
         2.0f * k3_altitude + k4_altitude);

    next_state.vertical_velocity_fps =
        state.vertical_velocity_fps +
        dt / 6.0f *
        (k1_velocity + 2.0f * k2_velocity +
         2.0f * k3_velocity + k4_velocity);

    return next_state;
}

}  // namespace control::physics