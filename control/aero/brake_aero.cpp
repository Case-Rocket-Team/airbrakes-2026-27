#include "brake_aero.hpp"

namespace control {
namespace aero {

f32 drag_coefficient(
    const AeroParameters& parameters,
    f32 brake_extension
) {
    return parameters.base_drag_coefficient
        + brake_extension * parameters.max_brake_delta_cd;
}

}  // namespace aero
}  // namespace control
