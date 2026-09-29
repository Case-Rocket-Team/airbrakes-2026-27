#ifndef CONTROL_AERO_BRAKE_AERO_HPP
#define CONTROL_AERO_BRAKE_AERO_HPP

#include "../../base/base.h"

namespace control {
namespace aero {

struct AeroParameters {
    f32 base_drag_coefficient;
    f32 max_brake_delta_cd;
};

/**
 * Returns the effective drag coefficient for a given brake extension.
 *
 * brake_extension is normalized:
 *   0.0 = fully retracted
 *   1.0 = fully extended
 *
 * This initial model assumes brake drag increases linearly with extension.
 * Aerodynamic parameters are supplied by the caller.
 */
f32 drag_coefficient(
    const AeroParameters& parameters,
    f32 brake_extension
);

}  // namespace aero
}  // namespace control

#endif  // CONTROL_AERO_BRAKE_AERO_HPP
