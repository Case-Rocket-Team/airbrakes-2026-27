#include "control/tests/closed_loop_simulation.hpp"

#include <cassert>

int main() {
    using namespace control::tests;

    constexpr f32 target_apogee_ft = 8500.0f;

    const ClosedLoopResult result = run_closed_loop(
        target_apogee_ft,
        "closed_loop_reachable.csv"
    );

    // Regression checks for the reachable 8,500 ft target.
    assert(result.maximum_extension > 0.0f);
    assert(result.apogee_error < 10.0f);

    print_closed_loop_result(result, target_apogee_ft);

    return 0;
}
