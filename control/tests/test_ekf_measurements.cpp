#include "control/ekf_adapter.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    ekf_settings settings{};

    settings.baro_var_ft2 = 100.0f;

    settings.gnss_covar_ft2[0] = 18.0f;
    settings.gnss_covar_ft2[4] = 18.0f;
    settings.gnss_covar_ft2[8] = 200.0f;

    // Test 1: Barometer correction.
    extended_kalman_filter baro_ekf{};
    ekf_init(&baro_ekf, &settings, 0);

    baro_ekf.nominal_state.pos_ft.z = 3000.0f;

    const float baro_before = baro_ekf.nominal_state.pos_ft.z;

    ekf_inject_baro(&baro_ekf, 3100.0f, 0);

    const float baro_after = baro_ekf.nominal_state.pos_ft.z;

    assert(std::isfinite(baro_after));
    assert(std::fabs(3100.0f - baro_after) <
           std::fabs(3100.0f - baro_before));

    // Test 2: GNSS correction.
    extended_kalman_filter gnss_ekf{};
    ekf_init(&gnss_ekf, &settings, 0);

    gnss_ekf.nominal_state.pos_ft.z = 3000.0f;

    const float gnss_before = gnss_ekf.nominal_state.pos_ft.z;

    ekf_inject_gnss(
        &gnss_ekf,
        vec3f{0.0f, 0.0f, 3100.0f},
        0
    );

    const float gnss_after = gnss_ekf.nominal_state.pos_ft.z;

    assert(std::isfinite(gnss_after));
    assert(std::fabs(3100.0f - gnss_after) <
           std::fabs(3100.0f - gnss_before));

    std::printf(
        "Barometer correction: %.3f -> %.3f ft\n",
        baro_before, baro_after
    );

    std::printf(
        "GNSS correction: %.3f -> %.3f ft\n",
        gnss_before, gnss_after
    );

    std::printf("EKF measurement tests passed.\n");
    return 0;
}
