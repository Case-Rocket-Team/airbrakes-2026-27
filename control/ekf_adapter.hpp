#ifndef CONTROL_EKF_ADAPTER_HPP
#define CONTROL_EKF_ADAPTER_HPP

#include "control/physics/dynamics.hpp"
#include "control/rocket_state.hpp"
#include "kalman_filter/extended_kalman_filter.h"

namespace control {

// Convert the EKF's world-frame position and velocity
// into the vertical state used by the apogee predictor.
//
// Assumptions:
// - World-frame +Z is upward.
// - EKF and MPC use the same altitude reference.
// - Position is in feet and velocity is in ft/s.
inline physics::VerticalState vertical_state_from_ekf(
    const extended_kalman_filter& ekf
) {
    return physics::VerticalState{
        ekf.nominal_state.pos_ft.z,
        ekf.nominal_state.vel_fps.z
    };
}

// Convert EKF output to the controller-facing state interface.
inline RocketState rocket_state_from_ekf(
    const extended_kalman_filter& ekf
) {
    return RocketState{
        ekf.nominal_state.pos_ft.z,
        ekf.nominal_state.vel_fps.z,
        ekf.state_time_us
    };
}

}  // namespace control

#endif  // CONTROL_EKF_ADAPTER_HPP
