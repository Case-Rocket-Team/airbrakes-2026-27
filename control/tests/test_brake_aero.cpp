#include <cassert>
#include <cmath>

#include "../aero/brake_aero.hpp"

namespace {

bool approximately_equal(f32 a, f32 b, f32 tolerance = 1e-6f) {
    return std::fabs(a - b) <= tolerance;
}

}  // namespace

int main() {
    const control::aero::AeroParameters parameters{
        0.30f,
        0.20f
    };

    // Retracted brakes should use the base drag coefficient.
    assert(approximately_equal(
        control::aero::drag_coefficient(parameters, 0.0f),
        0.30f
    ));

    // Half extension should add half of the available brake drag.
    assert(approximately_equal(
        control::aero::drag_coefficient(parameters, 0.5f),
        0.40f
    ));

    // Full extension should add the full brake drag contribution.
    assert(approximately_equal(
        control::aero::drag_coefficient(parameters, 1.0f),
        0.50f
    ));

    return 0;
}
