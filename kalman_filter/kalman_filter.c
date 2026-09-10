
#include "kalman_filter.h"

void kf_predict(kalman_filter* kf, kf_control_input* control, f32 dt) {
    // Commonly denoted F
    f32 state_transition[KF_STATE_DIM * KF_STATE_DIM] = {
        1.0f, dt, -0.5f * dt * dt,
        0.0f, 1.0f, -dt, 
        0.0f, 0.0f, 1.0f,
    };

    // Commonly denoted G
    f32 control_model[KF_STATE_DIM * 1] = {
        0.5f * dt * dt,
        dt,
        0.0f,
    };

    // x_n+1|n = F * x_n|n + G * u_n
    {
        f32 next_state[KF_STATE_DIM] = { 0 };

        matmul(
            false, false,
            KF_STATE_DIM, 1, KF_STATE_DIM,
            1.0f, state_transition, kf->state.vec.v, 
            0.0f, next_state
        );

        matmul(
            false, false,
            KF_STATE_DIM, 1, 1,
            1.0f, control_model, control->v, 
            1.0f, next_state
        );

        memcpy(kf->state.vec.v, next_state, sizeof(next_state));
    }

    // P_n+1|n = F * P_n|n * F^T + Q
    {
        // Filling this in the with process noise Q
        // This is computed here because it is dependent on dt
        f32 v_a = kf->accel_stddev_fps2 * kf->accel_stddev_fps2;
        f32 v_b = kf->accel_bias_stddev_fps2 * kf->accel_bias_stddev_fps2;
        f32 dt2 = dt * dt;
        f32 next_covariance[KF_STATE_DIM * KF_STATE_DIM] = {
            0.25f * dt2 * dt2 * v_a, 0.5f * dt2 * dt * v_a, 0.0f,
            0.5f * dt2 * dt * v_a, dt2 * v_a, 0.0f,
            0.0f, 0.0f, v_b,
        };

        // Intermediate matrix equal to F * P_n|n
        f32 FP[KF_STATE_DIM * KF_STATE_DIM] = { 0 };

        matmul(
            false, false,
            KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM,
            1.0f, state_transition, kf->state.covariance,
            0.0f, FP
        );
        
        matmul(
            false, true,
            KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM,
            1.0f, FP, state_transition,
            1.0f, next_covariance
        );

        memcpy(kf->state.covariance, next_covariance, sizeof(next_covariance));
    }
}

void kf_update(kalman_filter* kf, kf_measure* measure) {
    // Commonly denoted H
    f32 observation_model[KF_MEASURE_DIM * KF_STATE_DIM] = {
        1.0f, 0.0f, 0.0f
    };

    // y = z - Hx_n|n-1
    f32 innovation_vec[KF_MEASURE_DIM];
    memcpy(innovation_vec, measure->v, sizeof(innovation_vec));
    matmul(
        false, false,
        KF_MEASURE_DIM, 1, KF_STATE_DIM,
        -1.0f, observation_model, kf->state.vec.v,
        1.0f, innovation_vec
    );

    f32 kalman_gain[KF_STATE_DIM * KF_MEASURE_DIM] = { 0 };
    {
        f32 innovation_covariance_inv[KF_MEASURE_DIM * KF_MEASURE_DIM] = { 0 };

        // Initialize with identity
        for (u32 i = 0; i < KF_MEASURE_DIM; i++) {
            innovation_covariance_inv[i + i * KF_MEASURE_DIM] = 1.0f;
        }

        // Stores P * observation_model^T
        // This is used to get the innovation covariance as well as the kalman
        // gain
        f32 PH_T[KF_STATE_DIM * KF_MEASURE_DIM] = { 0 };

        matmul(
            false, true,
            KF_STATE_DIM, KF_MEASURE_DIM, KF_STATE_DIM,
            1.0f, kf->state.covariance, observation_model,
            0.0f, PH_T
        );

        // S = HPH^T + R
        // Where R is the measurement covariance
        f32 innovation_covariance[KF_MEASURE_DIM * KF_MEASURE_DIM];
        memcpy(
            innovation_covariance, kf->measure_covariance,
            sizeof(innovation_covariance)
        );

        matmul(
            false, false,
            KF_MEASURE_DIM, KF_MEASURE_DIM, KF_STATE_DIM,
            1.0f, observation_model, PH_T,
            1.0f, innovation_covariance
        );

        linear_solve(
            KF_MEASURE_DIM, KF_MEASURE_DIM,
            innovation_covariance, innovation_covariance_inv
        );

        // K = PH^T * S^-1
        matmul(
            false, false,
            KF_STATE_DIM, KF_MEASURE_DIM, KF_MEASURE_DIM,
            1.0f, PH_T, innovation_covariance_inv,
            0.0f, kalman_gain
        );
    }

    // x_n|n = x_n|n-1 + Ky
    matmul(
        false, false,
        KF_STATE_DIM, 1, KF_MEASURE_DIM,
        1.0f, kalman_gain, innovation_vec,
        1.0f, kf->state.vec.v
    );

    // Updating covariance according to
    // P_n|n = (I - KH) * P_n|n-1 * (I - KH)^T + KRK^T
    {
        f32 cov_factor[KF_STATE_DIM * KF_STATE_DIM] = { 0 };

        // Initializing with identity
        for (u32 i = 0; i < KF_STATE_DIM; i++) {
            cov_factor[i + i * KF_STATE_DIM] = 1.0f;
        }

        // I - KH
        matmul(
            false, false,
            KF_STATE_DIM, KF_STATE_DIM, KF_MEASURE_DIM,
            -1.0f, kalman_gain, observation_model,
            1.0f, cov_factor
        );

        // Temporarily stores (I - KH) * P_n|n-1 and later KR
        f32 leftmul[KF_STATE_DIM * MAX(KF_MEASURE_DIM, KF_STATE_DIM)] = { 0 };

        // (I - KH) * P_n|n-1
        matmul(
            false, false,
            KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM,
            1.0f, cov_factor, kf->state.covariance,
            0.0f, leftmul
        );

        // P_n|n = (I - KH) * P_n|n-1 * (I - KH)^T
        // (Just the first half of the covariaince update)
        matmul(
            false, true,
            KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM,
            1.0f, leftmul, cov_factor,
            0.0f, kf->state.covariance
        );

        // K*R
        matmul(
            false, false,
            KF_STATE_DIM, KF_MEASURE_DIM, KF_MEASURE_DIM,
            1.0f, kalman_gain, kf->measure_covariance,
            0.0f, leftmul
        );

        // P_n|n += KRK^T
        matmul(
            false, true,
            KF_STATE_DIM, KF_STATE_DIM, KF_MEASURE_DIM,
            1.0f, leftmul, kalman_gain,
            1.0f, kf->state.covariance
        );
    }
}

