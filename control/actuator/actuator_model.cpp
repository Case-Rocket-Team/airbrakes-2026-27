
#include "actuator_model.hpp"

#include <algorithm>
#include <cmath>

namespace control::actuator {

f32 next_extension(
    f32 current_extension,
    f32 commanded_extension,
    f32 dt,
    const ActuatorParameters& parameters
) {
    const f32 current = std::clamp(
        current_extension,
        0.0f,
        1.0f
    );

    const f32 target = std::clamp(
        commanded_extension,
        0.0f,
        1.0f
    );

    // Invalid timestep or actuator rate: no movement.
    if (!std::isfinite(dt) || dt <= 0.0f ||
        !std::isfinite(parameters.max_slew_rate) ||
        parameters.max_slew_rate <= 0.0f) {
        return current;
    }

    // Maximum distance the actuator can travel this timestep.
    const f32 max_change = parameters.max_slew_rate * dt;

    // Limit movement toward the commanded position.
    const f32 change = std::clamp(
        target - current,
        -max_change,
        max_change
    );

    // Ensure extension remains within physical bounds.
    return std::clamp(
        current + change,
        0.0f,
        1.0f
    );
}

} // namespace control::actuator
