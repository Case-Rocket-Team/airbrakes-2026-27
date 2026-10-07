#include "control/physics/atmosphere.hpp"

#include <cmath>

namespace control::physics {

namespace {

constexpr f32 SEA_LEVEL_TEMPERATURE_R = 518.67f;
constexpr f32 SEA_LEVEL_PRESSURE_LBF_FT2 = 2116.22f;
constexpr f32 TEMPERATURE_LAPSE_RATE_R_PER_FT = 0.00356616f;

constexpr f32 GRAVITY_FPS2 = 32.174f;
constexpr f32 AIR_GAS_CONSTANT_FT_LBF_PER_SLUG_R = 1716.59f;

}  // namespace

f32 air_density(f32 altitude_ft) {
    const f32 temperature_r =
        SEA_LEVEL_TEMPERATURE_R -
        TEMPERATURE_LAPSE_RATE_R_PER_FT * altitude_ft;

    const f32 exponent =
        GRAVITY_FPS2 /
        (AIR_GAS_CONSTANT_FT_LBF_PER_SLUG_R * TEMPERATURE_LAPSE_RATE_R_PER_FT);

    const f32 pressure_lbf_ft2 =
        SEA_LEVEL_PRESSURE_LBF_FT2 *
        std::pow(temperature_r / SEA_LEVEL_TEMPERATURE_R, exponent);

    const f32 density_slug_ft3 =
        pressure_lbf_ft2 /
        (AIR_GAS_CONSTANT_FT_LBF_PER_SLUG_R * temperature_r);

    return density_slug_ft3;
}

}  // namespace control::physics