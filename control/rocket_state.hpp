#ifndef CONTROL_ROCKET_STATE_HPP
#define CONTROL_ROCKET_STATE_HPP

namespace control {

/*
 * Controller-facing rocket state.
 *
 * Kept separate from the EKF state so the control algorithm does not depend
 * on a specific state estimator implementation. The EKF output is converted
 * to this interface before being passed to the controller.
 */
struct RocketState {
    float altitude_ft;
    float vertical_velocity_fps;
};

}  // namespace control

#endif  // CONTROL_ROCKET_STATE_HPP