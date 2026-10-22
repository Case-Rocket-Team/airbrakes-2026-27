#include "control/controller/slew_mpc.hpp"
#include "control/actuator/actuator_model.hpp"
#include "control/aero/brake_aero.hpp"
#include "control/physics/dynamics.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    using namespace control;

    // Simulated initial conditions at motor burnout.
    const physics::VerticalState initial_state{
        3000.0f,  // Altitude [ft]
        800.0f    // Vertical velocity [ft/s]
    };

    // Illustrative rocket parameters, not flight-validated.
    const physics::RocketParameters rocket_parameters{
        0.5f,  // Mass [slug]
        0.1f   // Reference area [ft^2]
    };

    // Illustrative aerodynamic parameters.
    const aero::AeroParameters aero_parameters{
        0.4f,  // Base drag coefficient
        0.8f   // Maximum additional brake drag coefficient
    };

    // Illustrative actuator maximum slew rate.
    const actuator::ActuatorParameters actuator_parameters{
        0.5f  // Maximum extension change per second
    };

    const MpcParameters mpc_parameters{
        8500.0f,  // Target apogee [ft]
        0.05f,     // Prediction timestep [s]
        0.1f       // Candidate extension increment
    };

    constexpr f32 simulation_dt = 0.05f;

    physics::VerticalState state = initial_state;
    f32 current_extension = 0.0f;
    f32 maximum_extension = 0.0f;

    f32 elapsed_time = 0.0f;
    constexpr int max_steps = 2000;

    for (
        int step = 0;
        step < max_steps && state.vertical_velocity_fps > 0.0f;
        ++step
    ) {
        // Select the best brake command.
        const BrakeCommand command = slew_mpc(
            state,
            rocket_parameters,
            aero_parameters,
            mpc_parameters,
            current_extension,
            actuator_parameters
        );

        // Simulate physical actuator movement.
        const f32 next_extension = actuator::next_extension(
            current_extension,
            command.extension,
            simulation_dt,
            actuator_parameters
        );

        // Verify extension and slew constraints.
        assert(next_extension >= 0.0f);
        assert(next_extension <= 1.0f);

        assert(
            std::fabs(next_extension - current_extension)
            <= actuator_parameters.max_slew_rate * simulation_dt + 1e-5f
        );

        // Approximate drag during actuator movement.
        const f32 average_extension =
            0.5f * (current_extension + next_extension);

        const f32 drag_coefficient = aero::drag_coefficient(
            aero_parameters,
            average_extension
        );

        // Propagate the rocket's physical state.
        state = physics::propagate_vertical(
            state,
            rocket_parameters,
            drag_coefficient,
            simulation_dt
        );

        current_extension = next_extension;

        if (current_extension > maximum_extension) {
            maximum_extension = current_extension;
        }

        elapsed_time += simulation_dt;

        // Print one sample approximately every second.
        if (step % 20 == 0) {
            std::printf(
                "t=%.2f s | altitude=%.1f ft | velocity=%.1f ft/s | extension=%.2f\n",
                elapsed_time,
                state.altitude_ft,
                state.vertical_velocity_fps,
                current_extension
            );
        }
    }

    assert(state.vertical_velocity_fps <= 0.0f);

    const f32 apogee_error =
        std::fabs(state.altitude_ft - mpc_parameters.target_apogee_ft);
    // Regression checks for the reachable 8,500 ft target.
    assert(maximum_extension > 0.0f);
    assert(apogee_error < 10.0f);

    std::printf("\nSimulation complete.\n");
    std::printf("Apogee: %.1f ft\n", state.altitude_ft);
    std::printf("Target: %.1f ft\n", mpc_parameters.target_apogee_ft);
    std::printf("Absolute error: %.1f ft\n", apogee_error);
    std::printf("Elapsed time: %.2f s\n", elapsed_time);

    return 0;
}