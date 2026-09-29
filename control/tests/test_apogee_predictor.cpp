#include "control/predictor/apogee_predictor.hpp"
#include "control/tests/test_common.hpp"

#include <cassert>


int main() {
    // Arbitrary test parameters, not values for the actual rocket.
    const control::physics::RocketParameters parameters{
        2.0f,  // mass [slug]
        1.0f   // reference area [ft^2]
    };

    const control::physics::VerticalState initial_state{
        1000.0f,  // altitude [ft]
        100.0f    // vertical velocity [ft/s]
    };

    constexpr f32 gravity_fps2 = 32.174f;

    // Exact analytical apogee for vertical motion with gravity only.
    const f32 expected_apogee =
        initial_state.altitude_ft +
        (initial_state.vertical_velocity_fps *
         initial_state.vertical_velocity_fps) /
        (2.0f * gravity_fps2);

    const f32 predicted_apogee =
        control::predictor::predict_apogee(
            initial_state,
            parameters,
            0.0f,   // no drag
            0.01f   // prediction timestep [s]
        );

    assert(approximately_equal(
        predicted_apogee,
        expected_apogee,
        0.1f
    ));

    // Check prediction accuracy with a larger timestep.
    const f32 predicted_apogee_large_dt =
        control::predictor::predict_apogee(
            initial_state,
            parameters,
            0.0f,
            0.1f
        );

    assert(approximately_equal(
        predicted_apogee_large_dt,
        expected_apogee,
        1.0f
    ));

    // Prediction must not modify the input state.
    assert(approximately_equal(
        initial_state.altitude_ft,
        1000.0f,
        0.000001f
    ));

    assert(approximately_equal(
        initial_state.vertical_velocity_fps,
        100.0f,
        0.000001f
    ));

    // Aerodynamic drag should reduce predicted apogee.
    const f32 drag_apogee =
        control::predictor::predict_apogee(
            initial_state,
            parameters,
            0.5f,
            0.01f
        );

    assert(drag_apogee < predicted_apogee);
    assert(drag_apogee > initial_state.altitude_ft);

        // Arbitrary aerodynamic test parameters, not actual rocket values.
    const control::aero::AeroParameters aero_parameters{
        0.30f,  // base drag coefficient
        0.20f   // maximum brake drag contribution
    };

    const f32 retracted_apogee =
        control::predictor::predict_apogee(
            initial_state,
            parameters,
            aero_parameters,
            0.0f,
            0.01f
        );

    const f32 half_extension_apogee =
        control::predictor::predict_apogee(
            initial_state,
            parameters,
            aero_parameters,
            0.5f,
            0.01f
        );

    const f32 full_extension_apogee =
        control::predictor::predict_apogee(
            initial_state,
            parameters,
            aero_parameters,
            1.0f,
            0.01f
        );

    // More brake extension should produce a lower predicted apogee.
    assert(retracted_apogee > half_extension_apogee);
    assert(half_extension_apogee > full_extension_apogee);

    return 0;
}
