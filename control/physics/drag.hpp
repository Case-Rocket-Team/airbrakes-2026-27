#ifndef CONTROL_PHYSICS_DRAG_HPP
#define CONTROL_PHYSICS_DRAG_HPP

#include "base/base.h"

namespace control::physics {

/*
 * Returns the magnitude of aerodynamic drag force.
 *
 * Inputs:
 *   air_density_slug_ft3    air density [slug/ft^3]
 *   airspeed_fps            airspeed magnitude [ft/s]
 *   drag_coefficient        dimensionless drag coefficient
 *   reference_area_ft2      aerodynamic reference area [ft^2]
 *
 * Output:
 *   drag force magnitude [lbf]
 *
 * Direction is intentionally not handled here. The dynamics model is
 * responsible for applying drag opposite the direction of motion.
 */
f32 drag_force(
    f32 air_density_slug_ft3,
    f32 airspeed_fps,
    f32 drag_coefficient,
    f32 reference_area_ft2
);

}  // namespace control::physics

#endif  // CONTROL_PHYSICS_DRAG_HPP
