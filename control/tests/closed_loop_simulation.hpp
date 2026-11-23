#pragma once

#include "control/controller/slew_mpc.hpp"
#include "control/actuator/actuator_model.hpp"
#include "control/aero/brake_aero.hpp"
#include "control/physics/dynamics.hpp"
#include "control/predictor/apogee_predictor.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace control::tests {

struct ClosedLoopResult {
    physics::VerticalState final_state;
    f32 maximum_extension;
    f32 elapsed_time;
    f32 apogee_error;
};

inline ClosedLoopResult run_closed_loop(
    f32 target_apogee_ft,
    const char* csv_filename
) {
    const physics::VerticalState initial_state{
        3000.0f,
        800.0f
    };

    // Illustrative parameters, not flight-validated.
    const physics::RocketParameters rocket_parameters{
        0.5f,
        0.1f
    };

    const aero::AeroParameters aero_parameters{
        0.4f,
        0.8f
    };

    const actuator::ActuatorParameters actuator_parameters{
        0.5f
    };

    const MpcParameters mpc_parameters{
        target_apogee_ft,
        0.05f,
        0.1f
    };

    constexpr f32 simulation_dt = 0.05f;
    constexpr int max_steps = 2000;

    physics::VerticalState state = initial_state;
    f32 current_extension = 0.0f;
    f32 maximum_extension = 0.0f;
    f32 elapsed_time = 0.0f;

    std::ofstream csv(csv_filename);
    assert(csv.is_open());

    csv << "time_s,altitude_ft,velocity_fps,"
        << "predicted_apogee_ft,target_apogee_ft,"
        << "commanded_extension,actual_extension\n";

    for (
        int step = 0;
        step < max_steps && state.vertical_velocity_fps > 0.0f;
        ++step
    ) {
        const BrakeCommand command = slew_mpc(
            state,
            rocket_parameters,
            aero_parameters,
            mpc_parameters,
            current_extension,
            actuator_parameters
        );

        const f32 predicted_apogee = predictor::predict_apogee(
            state,
            rocket_parameters,
            aero_parameters,
            current_extension,
            command.extension,
            mpc_parameters.prediction_dt,
            actuator_parameters
        );

        csv << elapsed_time << ","
            << state.altitude_ft << ","
            << state.vertical_velocity_fps << ","
            << predicted_apogee << ","
            << mpc_parameters.target_apogee_ft << ","
            << command.extension << ","
            << current_extension << "\n";

        const f32 next_extension = actuator::next_extension(
            current_extension,
            command.extension,
            simulation_dt,
            actuator_parameters
        );

        assert(next_extension >= 0.0f);
        assert(next_extension <= 1.0f);
        assert(
            std::fabs(next_extension - current_extension)
            <= actuator_parameters.max_slew_rate * simulation_dt + 1e-5f
        );

        const f32 average_extension =
            0.5f * (current_extension + next_extension);

        const f32 drag_coefficient = aero::drag_coefficient(
            aero_parameters,
            average_extension
        );

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

        if (step % 20 == 0) {
            std::printf(
                "t=%.2f s | altitude=%.1f ft | velocity=%.1f ft/s | extension=%.2f | predicted=%.1f ft | command=%.2f\n",
                elapsed_time,
                state.altitude_ft,
                state.vertical_velocity_fps,
                current_extension,
                predicted_apogee,
                command.extension
            );
        }
    }

    assert(state.vertical_velocity_fps <= 0.0f);

    return {
        state,
        maximum_extension,
        elapsed_time,
        std::fabs(state.altitude_ft - target_apogee_ft)
    };
}

inline void print_closed_loop_result(
    const ClosedLoopResult& result,
    f32 target_apogee_ft
) {
    std::printf("\nSimulation complete.\n");
    std::printf("Apogee: %.1f ft\n", result.final_state.altitude_ft);
    std::printf("Target: %.1f ft\n", target_apogee_ft);
    std::printf("Absolute error: %.1f ft\n", result.apogee_error);
    std::printf("Elapsed time: %.2f s\n", result.elapsed_time);
}

}  // namespace control::tests
