
#pragma once

#include "base/base.h"

namespace control::actuator {

struct ActuatorParameters {
    // Maximum change in normalized extension per second.
    // Example: 0.2f = 20% extension per second.
    f32 max_slew_rate;
};

// Predict brake extension after dt seconds.
// Extension ranges from 0.0 (retracted) to 1.0 (fully extended).
f32 next_extension(
    f32 current_extension,
    f32 commanded_extension,
    f32 dt,
    const ActuatorParameters& parameters
);

} // namespace control::actuator
