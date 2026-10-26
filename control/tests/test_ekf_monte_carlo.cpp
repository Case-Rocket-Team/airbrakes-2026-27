#include "control/ekf_adapter.hpp"
#include "control/controller/slew_mpc.hpp"
#include "control/actuator/actuator_model.hpp"
#include "control/aero/brake_aero.hpp"
#include "control/physics/dynamics.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

float run_simulation(unsigned int seed) {
    using namespace control;

    const physics::RocketParameters rocket{0.5f, 0.1f};
    const aero::AeroParameters aero{0.4f, 0.8f};
    const actuator::ActuatorParameters actuator{0.5f};
    const MpcParameters mpc{8500.0f, 0.05f, 0.1f};

    std::mt19937 rng(seed);

    std::normal_distribution<f32> baro_noise(0.0f, 3.0f);
    std::normal_distribution<f32> accel_noise(0.0f, 0.5f);

    constexpr f32 dt = 0.05f;
    constexpr u32 dt_us = 50000;
    constexpr int max_steps = 2000;

    physics::VerticalState truth{3000.0f, 800.0f};

    ekf_settings settings{};
    settings.accel_var_f2ps4 = 0.25f;
    settings.gyro_var_rad2ps2 = 1e-6f;
    settings.baro_var_ft2 = 9.0f;

    extended_kalman_filter ekf{};
    ekf_init(&ekf, &settings, 0);

    // Start with the known burnout state.
    ekf.nominal_state.pos_ft.z = truth.altitude_ft;
    ekf.nominal_state.vel_fps.z = truth.vertical_velocity_fps;

    f32 extension = 0.0f;
    f32 maximum_extension = 0.0f;

    // Establish initial IMU input.
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

        assert(std::isfinite(command.extension));
        assert(command.extension >= 0.0f);
        assert(command.extension <= 1.0f);

        const f32 next_extension = actuator::next_extension(
            extension, command.extension, dt, actuator
        );

        const f32 average_extension =
            0.5f * (extension + next_extension);

        const f32 cd = aero::drag_coefficient(
            aero, average_extension
        );

        const physics::VerticalState previous_truth = truth;

        truth = physics::propagate_vertical(
            truth, rocket, cd, dt
        );

        // Infer the average specific force over this simulation step.
        // EKF world-z is positive upward.
        const f32 world_acceleration =
            (truth.vertical_velocity_fps -
             previous_truth.vertical_velocity_fps) / dt;

        const f32 imu_specific_force =
            world_acceleration + 32.174f;

        extension = next_extension;

        if (extension > maximum_extension) {
            maximum_extension = extension;
        }

        const u32 timestamp =
            static_cast<u32>((step + 1) * dt_us);

        // Advance the EKF using the previous IMU sample.
        ekf_inject_imu(
            &ekf,
            vec3f{0.0f, 0.0f, imu_specific_force + accel_noise(rng)},
            vec3f{0.0f, 0.0f, 0.0f},
            timestamp
        );

        // Correct the estimate using a noisy barometer measurement.
        ekf_inject_baro(
            &ekf,
            truth.altitude_ft + baro_noise(rng),
            timestamp
        );
    }

    assert(truth.vertical_velocity_fps <= 0.0f);
    assert(maximum_extension > 0.0f);

    const f32 error =
        std::fabs(truth.altitude_ft - mpc.target_apogee_ft);

    return error;
}

int main() {
    constexpr int trials = 100;

    float total_error = 0.0f;
    float worst_error = 0.0f;

    for (int i = 0; i < trials; ++i) {
        const float error = run_simulation(42 + i);

        total_error += error;

        if (error > worst_error) {
            worst_error = error;
        }

        std::printf("Trial %d: %.2f ft error\n", i + 1, error);
    }

    std::printf("\nMean error: %.2f ft\n", total_error / trials);
    std::printf("Worst error: %.2f ft\n", worst_error);

    return 0;
}
