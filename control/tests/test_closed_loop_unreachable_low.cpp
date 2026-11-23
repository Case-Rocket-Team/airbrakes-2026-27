#include "control/tests/closed_loop_simulation.hpp"

#include <cassert>

int main() {
    using namespace control::tests;

    constexpr f32 target_apogee_ft = 6000.0f;

    const ClosedLoopResult result = run_closed_loop(
        target_apogee_ft,
        "closed_loop_unreachable_low.csv"
    );

    // Unreachable low target: brakes should extend almost fully.
    assert(result.maximum_extension > 0.99f);
    assert(result.final_state.altitude_ft > target_apogee_ft);

    print_closed_loop_result(result, target_apogee_ft);

    return 0;
}
