#ifndef CONTROL_PHYSICS_ATMOSPHERE_HPP
#define CONTROL_PHYSICS_ATMOSPHERE_HPP

#include "base/base.h"

namespace control::physics {

/*
 * Returns atmospheric density using a standard troposphere model.
 *
 * Input:
 *   altitude_ft     altitude above sea level [ft]
 *
 * Output:
 *   air density [slug/ft^3]
 *
 * Intended for the rocket's expected ascent range within the troposphere.
 */
f32 air_density(f32 altitude_ft);

}  // namespace control::physics

#endif  // CONTROL_PHYSICS_ATMOSPHERE_HPP