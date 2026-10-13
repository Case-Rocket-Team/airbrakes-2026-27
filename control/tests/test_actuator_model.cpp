
#include "../actuator/actuator_model.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using control::actuator::ActuatorParameters;
using control::actuator::next_extension;

bool approximately_equal(float a, float b) {
    return std::fabs(a - b) < 1e-5f;
}

int main() {
    const ActuatorParameters actuator{0.2f};

    // Test 1: Extension limited to 20% per second.
    assert(approximately_equal(
        next_extension(0.0f, 1.0f, 1.0f, actuator),
        0.2f
    ));

    // Test 2: Retraction limited to 20% per second.
    assert(approximately_equal(
        next_extension(1.0f, 0.0f, 1.0f, actuator),
        0.8f
    ));

    // Test 3: Reach target without overshooting.
    assert(approximately_equal(
        next_extension(0.4f, 0.5f, 1.0f, actuator),
        0.5f
    ));

    // Test 4: Clamp commanded extension to 100%.
    assert(approximately_equal(
        next_extension(0.9f, 1.5f, 1.0f, actuator),
        1.0f
    ));

    // Test 5: Clamp commanded extension to 0%.
    assert(approximately_equal(
        next_extension(0.1f, -0.5f, 1.0f, actuator),
        0.0f
    ));

    // Test 6: Zero timestep produces no movement.
    assert(approximately_equal(
        next_extension(0.3f, 1.0f, 0.0f, actuator),
        0.3f
    ));

    // Test 7: Negative timestep produces no movement.
    assert(approximately_equal(
        next_extension(0.3f, 1.0f, -1.0f, actuator),
        0.3f
    ));

    // Test 8: Zero slew rate produces no movement.
    assert(approximately_equal(
        next_extension(
            0.3f, 1.0f, 1.0f,
            ActuatorParameters{0.0f}
        ),
        0.3f
    ));

    // Test 9: NaN slew rate produces no movement.
    assert(approximately_equal(
        next_extension(
            0.3f, 1.0f, 1.0f,
            ActuatorParameters{
                std::numeric_limits<float>::quiet_NaN()
            }
        ),
        0.3f
    ));

    // Test 10: Multiple timesteps respect slew rate.
    float extension = 0.0f;

    for (int i = 0; i < 5; ++i) {
        extension = next_extension(
            extension, 1.0f, 0.5f, actuator
        );
    }

    assert(approximately_equal(extension, 0.5f));

    std::cout << "All actuator model tests passed.\n";
    return 0;
}
