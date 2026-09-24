#ifndef CONTROL_PHYSICS_DYNAMICS_HPP
#define CONTROL_PHYSICS_DYNAMICS_HPP

#include "base/base.h"

namespace control::physics {

/*
 * State used internally when propagating a vertical trajectory.
 *
 * Positive velocity is upward.
 */
struct VerticalState {
    f32 altitude_ft;
    f32 vertical_velocity_fps;
};

/*
 * Physical rocket parameters required by the vertical dynamics model.
 *
 * These values are supplied by the caller and are not hard-coded because
 * the physical rocket design may change.
 */
struct RocketParameters {
    f32 mass_slug;
    f32 reference_area_ft2;
};

/*
 * Propagates the rocket's vertical state forward by one timestep during
 * unpowered flight.
 *
 * Inputs:
 *   state               current vertical state
 *   parameters          current rocket physical parameters
 *   drag_coefficient    aerodynamic drag coefficient
 *   dt                  timestep [s]
 *
 * Returns:
 *   vertical state after dt seconds
 */
VerticalState propagate_vertical(
    const VerticalState& state,
    const RocketParameters& parameters,
    f32 drag_coefficient,
    f32 dt
);

}  // namespace control::physics

#endif  // CONTROL_PHYSICS_DYNAMICS_HPP