#include "control/tests/closed_loop_simulation.hpp"

#include <cassert>

int main() {
    using namespace control::tests;

    constexpr f32 target_apogee_ft = 10000.0f;

    const ClosedLoopResult result = run_closed_loop(
        target_apogee_ft,
        "closed_loop_unreachable_high.csv"
    );

    // Unreachable high target: brakes should remain retracted.
    assert(result.maximum_extension <= 1e-5f);
    assert(result.final_state.altitude_ft < target_apogee_ft);

    print_closed_loop_result(result, target_apogee_ft);

    return 0;
}
