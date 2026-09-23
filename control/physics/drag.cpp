#include "control/physics/drag.hpp"

namespace control::physics {

f32 drag_force(
    f32 air_density_slug_ft3,
    f32 airspeed_fps,
    f32 drag_coefficient,
    f32 reference_area_ft2
) {
    return 0.5f *
           air_density_slug_ft3 *
           airspeed_fps *
           airspeed_fps *
           drag_coefficient *
           reference_area_ft2;
}

}  // namespace control::physics