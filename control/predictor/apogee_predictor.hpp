#ifndef CONTROL_PREDICTOR_APOGEE_PREDICTOR_HPP
#define CONTROL_PREDICTOR_APOGEE_PREDICTOR_HPP

#include "base/base.h"
#include "control/physics/dynamics.hpp"
#include "control/aero/brake_aero.hpp"

namespace control::predictor {

/*
 * Predicts apogee by numerically propagating an unpowered vertical
 * trajectory until upward velocity reaches zero.
 *
 * Inputs:
 *   initial_state       current altitude and vertical velocity
 *   parameters          current rocket physical parameters
 *   drag_coefficient    aerodynamic drag coefficient
 *   dt                  prediction timestep [s]
 *
 * Output:
 *   predicted apogee altitude [ft]
 *
 * Rocket-specific physical values are supplied by the caller and are
 * not hard-coded into the predictor.
 */
f32 predict_apogee(
    const physics::VerticalState& initial_state,
    const physics::RocketParameters& parameters,
    f32 drag_coefficient,
    f32 dt
);

/*
 * Predicts apogee for a fixed brake extension.
 *
 * The brake position is assumed to remain constant for the duration
 * of the prediction.
 */
f32 predict_apogee(
    const physics::VerticalState& initial_state,
    const physics::RocketParameters& parameters,
    const aero::AeroParameters& aero_parameters,
    f32 brake_extension,
    f32 dt
);

}  // namespace control::predictor

#endif  // CONTROL_PREDICTOR_APOGEE_PREDICTOR_HPP
