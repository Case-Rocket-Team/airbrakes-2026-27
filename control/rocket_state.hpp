#ifndef CONTROL_ROCKET_STATE_HPP
#define CONTROL_ROCKET_STATE_HPP

#include "../base/base.h"

namespace control {

/*
 * Controller-facing rocket state.
 *
 * Kept separate from the EKF state so the control algorithm does not depend
 * on a specific state estimator implementation. The EKF output is converted
 * to this interface before being passed to the controller.
 */
struct RocketState {
    f32 altitude_ft;
    f32 vertical_velocity_fps;
    u32 timestamp_us;
};

}  // namespace control

#endif  // CONTROL_ROCKET_STATE_HPP