
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
            1.0f, control_model, control->vec.v, 
            1.0f, next_state
        );

        memcpy(kf->state.vec.v, next_state, sizeof(next_state));
    }

    // P_n+1|n = F * P_n|n * F^T + Q
    {
        f32 next_covariance[KF_STATE_DIM * KF_STATE_DIM] = { 0 };

        matmul(
            false, false,
            KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM,
            1.0f, state_transition, kf->state.covariance,
            0.0f, next_covariance
        );
        
        matmul(
            false, true,
            KF_STATE_DIM, KF_STATE_DIM, KF_STATE_DIM,
            1.0f, kf->state.covariance, state_transition,
            1.0f, next_covariance
        );

        for (u32 i = 0; i < KF_STATE_DIM * KF_STATE_DIM; i++) {
            next_covariance[i] += control->covariance[i];
        }

        memcpy(kf->state.covariance, next_covariance, sizeof(next_covariance));
    }
}

void kf_update(kalman_filter* kf, kf_measure* measure) {
    f32 observation_model[KF_MEASURE_DIM * KF_STATE_DIM] = {
        1.0f, 0.0f, 0.0f
    };
}

