
#pragma once

#include "control/brake_command.hpp"
#include "control/controller/ideal_mpc.hpp"
#include "control/actuator/actuator_model.hpp"

namespace control {

/*
 * V1 MPC controller with actuator slew-rate constraints.
 *
 * Evaluates candidate brake commands using the slew-aware
 * apogee predictor.
 *
 * current_extension is normalized to [0, 1].
 * max_slew_rate is normalized extension per second.
 *
 * For V1, current_extension is supplied by the caller.
 * V2 will replace the assumed position with actuator feedback.
 */
BrakeCommand slew_mpc(
    const physics::VerticalState& state,
    const physics::RocketParameters& rocket_parameters,
    const aero::AeroParameters& aero_parameters,
    const MpcParameters& mpc_parameters,
    f32 current_extension,
    const actuator::ActuatorParameters& actuator_parameters
);

} // namespace control
