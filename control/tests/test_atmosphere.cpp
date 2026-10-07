#include "control/physics/atmosphere.hpp"
#include "control/tests/test_common.hpp"

#include <cassert>


int main() {
    // Standard sea-level atmospheric density.
    const f32 sea_level_density = control::physics::air_density(0.0f);

    assert(approximately_equal(
        sea_level_density,
        0.002377f,
        0.00001f
    ));

    // Air density should decrease as altitude increases.
    const f32 density_10k = control::physics::air_density(10000.0f);
    const f32 density_20k = control::physics::air_density(20000.0f);
    const f32 density_30k = control::physics::air_density(30000.0f);

    assert(sea_level_density > density_10k);
    assert(density_10k > density_20k);
    assert(density_20k > density_30k);

    return 0;
}