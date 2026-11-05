#include "control/tests/ekf_simulation.hpp"

#include <cstdio>

int main() {
    constexpr int trials = 100;

    float total_error = 0.0f;
    float worst_error = 0.0f;

    for (int i = 0; i < trials; ++i) {
        const control::tests::EkfSimulationConfig config{
            true,
            static_cast<unsigned int>(42 + i),
            nullptr,
            false
        };

        const auto result =
            control::tests::run_ekf_simulation(config);

        const float error = result.apogee_error;
        total_error += error;

        if (error > worst_error) {
            worst_error = error;
        }

        std::printf("Trial %d: %.2f ft error\n", i + 1, error);
    }

    std::printf("\nMean error: %.2f ft\n", total_error / trials);
    std::printf("Worst error: %.2f ft\n", worst_error);

    return 0;
}
