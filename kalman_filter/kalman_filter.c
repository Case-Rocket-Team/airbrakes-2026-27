
#include "kalman_filter.h"

void kf_predict(kalman_filter* kf, kf_control_input* control, f32 dt) {
    f32 state_transition[KF_STATE_DIM * KF_STATE_DIM] = {
        1.0f, dt, -0.5f * dt * dt,
        0.0f, 1.0f, -dt, 
        0.0f, 0.0f, 1.0f,
    };

    f32 control_model[KF_STATE_DIM * 1] = {
        0.5f * dt * dt,
        dt,
        0.0f,
    };
}

void kf_update(kalman_filter* kf, kf_measure* measure, f32 dt) {
}

