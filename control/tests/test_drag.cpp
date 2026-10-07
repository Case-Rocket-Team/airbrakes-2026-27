#include "control/physics/drag.hpp"
#include "control/tests/test_common.hpp"

#include <cassert>

namespace {

}  // namespace

int main() {
    constexpr f32 density = 0.002f;
    constexpr f32 drag_coefficient = 0.5f;
    constexpr f32 reference_area = 1.0f;

    // Zero airspeed should produce zero drag.
    const f32 zero_drag = control::physics::drag_force(
        density,
        0.0f,
        drag_coefficient,
        reference_area
    );

    assert(approximately_equal(zero_drag, 0.0f, 0.000001f));

    // Verify the drag equation against a known calculation.
    const f32 drag_100 = control::physics::drag_force(
        density,
        100.0f,
        drag_coefficient,
        reference_area
    );

    assert(approximately_equal(drag_100, 5.0f, 0.0001f));

    // Drag should scale with the square of airspeed.
    const f32 drag_200 = control::physics::drag_force(
        density,
        200.0f,
        drag_coefficient,
        reference_area
    );

    assert(approximately_equal(drag_200, 4.0f * drag_100, 0.0001f));

    return 0;
}