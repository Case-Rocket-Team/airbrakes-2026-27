#ifndef CONTROL_BRAKE_COMMAND_HPP
#define CONTROL_BRAKE_COMMAND_HPP

namespace control {

struct BrakeCommand {
    // Normalized brake extension: 0.0 = retracted, 1.0 = fully extended.
    float extension;
};

}  // namespace control

#endif  // CONTROL_BRAKE_COMMAND_HPP