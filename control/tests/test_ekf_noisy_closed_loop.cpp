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

int main() {
    using namespace control;

    const physics::RocketParameters rocket{0.5f, 0.1f};
    const aero::AeroParameters aero{0.4f, 0.8f};
    const actuator::ActuatorParameters actuator{0.5f};
    const MpcParameters mpc{8500.0f, 0.05f, 0.1f};

    std::mt19937 rng(42);

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
    f32 elapsed = 0.0f;

    std::ofstream csv("ekf_noisy_closed_loop.csv");
    assert(csv.is_open());

    csv << "time_s,true_altitude_ft,estimated_altitude_ft,"
           "true_velocity_fps,estimated_velocity_fps,"
           "commanded_extension,actual_extension\n";

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

        csv << elapsed << ","
            << truth.altitude_ft << ","
            << estimated.altitude_ft << ","
            << truth.vertical_velocity_fps << ","
            << estimated.vertical_velocity_fps << ","
            << command.extension << ","
            << extension << "\n";

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

        elapsed += dt;
        const u32 timestamp =
            static_cast<u32>((step + 1) * dt_us);

        // Advance the EKF using the previous IMU sample.
        ekf_inject_imu(
            &ekf,
            vec3f{0.0f, 0.0f, imu_specific_force + accel_noise(rng)},
            vec3f{0.0f, 0.0f, 0.0f},
            timestamp
        );

        // Correct the estimate using an ideal barometer.
        ekf_inject_baro(
            &ekf,
            truth.altitude_ft + baro_noise(rng),
            timestamp
        );

        if (step % 20 == 0) {
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

    const f32 error =
        std::fabs(truth.altitude_ft - mpc.target_apogee_ft);

    std::printf("\nNoisy EKF closed-loop simulation complete.\n");
    std::printf("Actual apogee: %.1f ft\n", truth.altitude_ft);
    std::printf("Target apogee: %.1f ft\n", mpc.target_apogee_ft);
    std::printf("Absolute error: %.1f ft\n", error);
    std::printf("Elapsed time: %.2f s\n", elapsed);

    return 0;
}
