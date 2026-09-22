#include "control/physics/atmosphere.hpp"

#include <cmath>

namespace control::physics {

namespace {

constexpr f32 FT_TO_M = 0.3048f;
constexpr f32 KG_M3_TO_SLUG_FT3 = 0.00194032f;

constexpr f32 SEA_LEVEL_TEMPERATURE_K = 288.15f;
constexpr f32 SEA_LEVEL_PRESSURE_PA = 101325.0f;
constexpr f32 TEMPERATURE_LAPSE_RATE_K_PER_M = 0.0065f;

constexpr f32 GRAVITY_MPS2 = 9.80665f;
constexpr f32 AIR_GAS_CONSTANT_J_PER_KG_K = 287.05f;

}  // namespace

f32 air_density(f32 altitude_ft) {
    const f32 altitude_m = altitude_ft * FT_TO_M;

    const f32 temperature_k =
        SEA_LEVEL_TEMPERATURE_K -
        TEMPERATURE_LAPSE_RATE_K_PER_M * altitude_m;

    const f32 exponent =
        GRAVITY_MPS2 /
        (AIR_GAS_CONSTANT_J_PER_KG_K * TEMPERATURE_LAPSE_RATE_K_PER_M);

    const f32 pressure_pa =
        SEA_LEVEL_PRESSURE_PA *
        std::pow(temperature_k / SEA_LEVEL_TEMPERATURE_K, exponent);

    const f32 density_kg_m3 =
        pressure_pa /
        (AIR_GAS_CONSTANT_J_PER_KG_K * temperature_k);

    return density_kg_m3 * KG_M3_TO_SLUG_FT3;
}

}  // namespace control::physics