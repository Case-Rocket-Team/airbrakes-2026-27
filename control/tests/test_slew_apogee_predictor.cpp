
#include "control/predictor/apogee_predictor.hpp"
#include "control/actuator/actuator_model.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    using namespace control;

    // Test-only parameters, not actual rocket constants.
    const physics::VerticalState state{
        5000.0f,  // Altitude (ft)
        500.0f    // Vertical velocity (ft/s)
    };

    // Check the field order against dynamics.hpp before running.
    const physics::RocketParameters rocket{
        1.0f,    // Mass (slugs)
        0.1f     // Reference area (ft²)
    };

    const aero::AeroParameters aero{
        0.4f,     // Base drag coefficient
        0.6f      // Additional drag at full extension
    };

    const f32 dt = 0.01f;

    // V0: instantaneous full deployment.
    const f32 instant = predictor::predict_apogee(
        state, rocket, aero, 1.0f, dt
    );

    // V1: slow deployment from 0% to 100%.
    const f32 slow = predictor::predict_apogee(
        state, rocket, aero,
        0.0f, 1.0f, dt,
        actuator::ActuatorParameters{0.1f}
    );

    assert(slow > instant);

    // Fast actuator should approximate instant deployment.
    const f32 fast = predictor::predict_apogee(
        state, rocket, aero,
        0.0f, 1.0f, dt,
        actuator::ActuatorParameters{10000.0f}
    );

    assert(std::fabs(fast - instant) < 1.0f);

    // Zero slew rate: extension stays at 0%.
    const f32 stationary = predictor::predict_apogee(
        state, rocket, aero,
        0.0f, 1.0f, dt,
        actuator::ActuatorParameters{0.0f}
    );

    const f32 retracted = predictor::predict_apogee(
        state, rocket, aero, 0.0f, dt
    );

    assert(std::fabs(stationary - retracted) < 1.0f);

    // Retraction from 100% toward 0%.
    const f32 retracting = predictor::predict_apogee(
        state, rocket, aero,
        1.0f, 0.0f, dt,
        actuator::ActuatorParameters{0.1f}
    );

    assert(retracting > instant);
    assert(retracting < retracted);

    std::cout << "All slew predictor tests passed.\n";
    return 0;
}
