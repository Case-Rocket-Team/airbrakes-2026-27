#include "../rocket_state.hpp"
#include "../brake_command.hpp"

#include <cassert>

int main() {
    control::RocketState state{
        8000.0f,
        500.0f,
        1000000u
    };

    control::BrakeCommand command{
        0.5f
    };

    assert(state.altitude_ft == 8000.0f);
    assert(state.vertical_velocity_fps == 500.0f);
    assert(state.timestamp_us == 1000000u);
    assert(command.extension == 0.5f);

    return 0;
}