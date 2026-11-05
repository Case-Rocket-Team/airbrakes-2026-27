#include "control/tests/ekf_simulation.hpp"

#include <cstdio>

int main() {
    const control::tests::EkfSimulationConfig config{
        false,
        42,
        "ekf_closed_loop.csv",
        true
    };

    const auto result =
        control::tests::run_ekf_simulation(config);

    std::printf("\nEKF closed-loop simulation complete.\n");
    std::printf("Actual apogee: %.1f ft\n",
                result.final_state.altitude_ft);
    std::printf("Target apogee: %.1f ft\n", 8500.0f);
    std::printf("Absolute error: %.1f ft\n",
                result.apogee_error);
    std::printf("Elapsed time: %.2f s\n",
                result.elapsed_time);

    return 0;
}
