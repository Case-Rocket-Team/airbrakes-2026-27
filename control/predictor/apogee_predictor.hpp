#ifndef CONTROL_PREDICTOR_APOGEE_PREDICTOR_HPP
#define CONTROL_PREDICTOR_APOGEE_PREDICTOR_HPP

#include "base/base.h"
#include "control/physics/dynamics.hpp"
#include "control/aero/brake_aero.hpp"
#include "control/actuator/actuator_model.hpp"

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


/*
 * Predicts apogee while accounting for actuator slew rate.
 *
 * The actuator gradually moves from current_extension toward
 * commanded_extension during the predicted coast.
 *
 * All extensions are normalized to [0, 1].
 */
f32 predict_apogee(
    const physics::VerticalState& initial_state,
    const physics::RocketParameters& parameters,
    const aero::AeroParameters& aero_parameters,
    f32 current_extension,
    f32 commanded_extension,
    f32 dt,
    const actuator::ActuatorParameters& actuator_parameters
);


}  // namespace control::predictor

#endif  // CONTROL_PREDICTOR_APOGEE_PREDICTOR_HPP
