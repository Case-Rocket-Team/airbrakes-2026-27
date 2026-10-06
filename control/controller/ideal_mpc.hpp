#pragma once

#include "../brake_command.hpp"
#include "../physics/dynamics.hpp"
#include "../aero/brake_aero.hpp"

namespace control {

struct MpcParameters {
    f32 target_apogee_ft;
    f32 prediction_dt;
    f32 extension_step;
};

BrakeCommand ideal_mpc(
    const physics::VerticalState& state,
    const physics::RocketParameters& rocket_parameters,
    const aero::AeroParameters& aero_parameters,
    const MpcParameters& mpc_parameters
);

} // namespace control
