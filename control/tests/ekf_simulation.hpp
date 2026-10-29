#pragma once

#include "control/ekf_adapter.hpp"
#include "control/controller/slew_mpc.hpp"
#include "control/actuator/actuator_model.hpp"
#include "control/aero/brake_aero.hpp"
#include "control/physics/dynamics.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>

namespace control::tests {

struct EkfSimulationConfig {
    bool noisy = false;
    unsigned int seed = 42;
    const char* csv_filename = nullptr;
    bool print_progress = false;
};

struct EkfSimulationResult {
    physics::VerticalState final_state;
    f32 maximum_extension;
    f32 elapsed_time;
    f32 apogee_error;

    f32 signed_apogee_error = 0.0f;
    int command_changes = 0;
    f32 total_command_change = 0.0f;
    f32 total_actuator_travel = 0.0f;
    int command_reversals = 0;
    int large_command_reversals = 0;
    f32 total_extension_time = 0.0f;
};


inline EkfSimulationResult run_ekf_simulation(
    const EkfSimulationConfig& config
) {
    // Illustrative parameters, not flight-validated.
    const physics::RocketParameters rocket{0.5f, 0.1f};
    const aero::AeroParameters aero{0.4f, 0.8f};
    const actuator::ActuatorParameters actuator{0.5f};
    const MpcParameters mpc{8500.0f, 0.05f, 0.1f};

    std::mt19937 rng(config.seed);
    std::normal_distribution<f32> baro_noise(0.0f, 3.0f);
    std::normal_distribution<f32> accel_noise(0.0f, 0.5f);

    constexpr f32 dt = 0.05f;
    constexpr u32 dt_us = 50000;
    constexpr int max_steps = 2000;

    physics::VerticalState truth{3000.0f, 800.0f};

    ekf_settings settings{};
    settings.accel_var_f2ps4 = config.noisy ? 0.25f : 0.01f;
    settings.gyro_var_rad2ps2 = 1e-6f;
    settings.baro_var_ft2 = config.noisy ? 9.0f : 1.0f;

    extended_kalman_filter ekf{};
    ekf_init(&ekf, &settings, 0);

    ekf.nominal_state.pos_ft.z = truth.altitude_ft;
    ekf.nominal_state.vel_fps.z = truth.vertical_velocity_fps;

    f32 extension = 0.0f;
    f32 maximum_extension = 0.0f;
    f32 elapsed = 0.0f;

    f32 previous_command = 0.0f;
    int command_changes = 0;
    f32 total_command_change = 0.0f;
    f32 total_actuator_travel = 0.0f;
    int command_reversals = 0;
    int large_command_reversals = 0;
    int previous_direction = 0;
    f32 total_extension_time = 0.0f;

    std::ofstream csv;
    if (config.csv_filename != nullptr) {
        csv.open(config.csv_filename);
        assert(csv.is_open());

        csv << "time_s,true_altitude_ft,estimated_altitude_ft,"
               "true_velocity_fps,estimated_velocity_fps,"
               "commanded_extension,actual_extension\n";
    }

    ekf_inject_imu(
        &ekf,
        vec3f{0.0f, 0.0f, 0.0f},
        vec3f{0.0f, 0.0f, 0.0f},
        0
    );

    for (int step = 0;
         step < max_steps && truth.vertical_velocity_fps > 0.0f;
         ++step) {

        const physics::VerticalState estimated =
            vertical_state_from_ekf(ekf);

        assert(std::isfinite(estimated.altitude_ft));
        assert(std::isfinite(estimated.vertical_velocity_fps));

        const BrakeCommand command = slew_mpc(
            estimated, rocket, aero, mpc, extension, actuator
        );

        const f32 signed_command_change =
            command.extension - previous_command;
        const f32 command_change =
            std::fabs(signed_command_change);

        if (command_change > 0.0001f) {
            ++command_changes;

            const int direction =
                signed_command_change > 0.0f ? 1 : -1;

            if (previous_direction != 0 &&
                direction != previous_direction) {
                ++command_reversals;

                if (command_change >= 0.30f) {
                    ++large_command_reversals;
                }
            }

            previous_direction = direction;
        }

        total_command_change += command_change;
        previous_command = command.extension;

        assert(std::isfinite(command.extension));
        assert(command.extension >= 0.0f);
        assert(command.extension <= 1.0f);

        if (csv.is_open()) {
            csv << elapsed << ","
                << truth.altitude_ft << ","
                << estimated.altitude_ft << ","
                << truth.vertical_velocity_fps << ","
                << estimated.vertical_velocity_fps << ","
                << command.extension << ","
                << extension << "\n";
        }

        const f32 next_extension = actuator::next_extension(
            extension, command.extension, dt, actuator
        );

        total_actuator_travel +=
            std::fabs(next_extension - extension);

        const f32 average_extension =
            0.5f * (extension + next_extension);

        total_extension_time += average_extension * dt;

        const f32 cd = aero::drag_coefficient(
            aero, average_extension
        );

        const physics::VerticalState previous_truth = truth;

        truth = physics::propagate_vertical(
            truth, rocket, cd, dt
        );

        const f32 world_acceleration =
            (truth.vertical_velocity_fps -
             previous_truth.vertical_velocity_fps) / dt;

        const f32 imu_specific_force =
            world_acceleration + 32.174f;

        extension = next_extension;

        if (extension > maximum_extension) {
            maximum_extension = extension;
        }

        elapsed += dt;
        const u32 timestamp =
            static_cast<u32>((step + 1) * dt_us);

        // Preserve the original sensor-update and RNG draw order.
        const f32 measured_acceleration = config.noisy
            ? imu_specific_force + accel_noise(rng)
            : imu_specific_force;

        ekf_inject_imu(
            &ekf,
            vec3f{0.0f, 0.0f, measured_acceleration},
            vec3f{0.0f, 0.0f, 0.0f},
            timestamp
        );

        const f32 measured_altitude = config.noisy
            ? truth.altitude_ft + baro_noise(rng)
            : truth.altitude_ft;

        ekf_inject_baro(
            &ekf,
            measured_altitude,
            timestamp
        );

        if (config.print_progress && step % 20 == 0) {
            const auto updated = vertical_state_from_ekf(ekf);

            std::printf(
                "t=%.2f | true=%.1f ft | EKF=%.1f ft | "
                "true_v=%.1f | EKF_v=%.1f | brake=%.2f\n",
                elapsed,
                truth.altitude_ft,
                updated.altitude_ft,
                truth.vertical_velocity_fps,
                updated.vertical_velocity_fps,
                extension
            );
        }
    }

    assert(truth.vertical_velocity_fps <= 0.0f);
    assert(maximum_extension > 0.0f);

    return {
        truth,
        maximum_extension,
        elapsed,
        std::fabs(truth.altitude_ft - mpc.target_apogee_ft),
        truth.altitude_ft - mpc.target_apogee_ft,
        command_changes,
        total_command_change,
        total_actuator_travel,
        command_reversals,
        large_command_reversals,
        total_extension_time
    };
}

}  // namespace control::tests
