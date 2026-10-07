#include "control/physics/dynamics.hpp"
#include "control/tests/test_common.hpp"

#include <cassert>

int main() {
    // Arbitrary test parameters. These are not values for the actual rocket.
    const control::physics::RocketParameters parameters{
        2.0f,  // mass [slug]
        1.0f   // reference area [ft^2]
    };

    // With Cd = 0, only gravity should affect the rocket.
    const control::physics::VerticalState no_drag_state{
        1000.0f,
        100.0f
    };

    const control::physics::VerticalState no_drag_next =
        control::physics::propagate_vertical(
            no_drag_state,
            parameters,
            0.0f,
            1.0f
        );

    assert(approximately_equal(
        no_drag_next.vertical_velocity_fps,
        67.826f,
        0.001f
    ));

    assert(approximately_equal(
        no_drag_next.altitude_ft,
        1083.913f,
        0.001f
    ));

    // Adding drag during ascent should reduce both velocity and altitude
    // relative to the gravity-only trajectory.
    const control::physics::VerticalState with_drag_next =
        control::physics::propagate_vertical(
            no_drag_state,
            parameters,
            0.5f,
            1.0f
        );

    assert(
        with_drag_next.vertical_velocity_fps <
        no_drag_next.vertical_velocity_fps
    );

    assert(
        with_drag_next.altitude_ft <
        no_drag_next.altitude_ft
    );

    // The original state should not be modified by propagation.
    assert(approximately_equal(
        no_drag_state.altitude_ft,
        1000.0f,
        0.000001f
    ));

    assert(approximately_equal(
        no_drag_state.vertical_velocity_fps,
        100.0f,
        0.000001f
    ));

    return 0;
}