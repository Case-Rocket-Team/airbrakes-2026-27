#include "../rocket_state.hpp"
#include "../brake_command.hpp"

#include <cassert>

int main() {
    control::RocketState state{
        8000.0f,
        500.0f
    };

    control::BrakeCommand command{
        0.5f
    };

    assert(state.altitude_ft == 8000.0f);
    assert(state.vertical_velocity_fps == 500.0f);
    assert(command.extension == 0.5f);

    return 0;
}