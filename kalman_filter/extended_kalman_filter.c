
#include "extended_kalman_filter.h"

void _ekf_fill_skew3(f32* m, vec3f v, u32 row_off, u32 col_off, u32 stride) {
    m[(row_off + 0) * stride + (col_off + 0)] = 0.0f;
    m[(row_off + 0) * stride + (col_off + 1)] = -v.z;
    m[(row_off + 0) * stride + (col_off + 2)] = v.y;

    m[(row_off + 1) * stride + (col_off + 0)] = v.z;
    m[(row_off + 1) * stride + (col_off + 1)] = 0.0f;
    m[(row_off + 1) * stride + (col_off + 2)] = -v.x;

    m[(row_off + 2) * stride + (col_off + 0)] = -v.y;
    m[(row_off + 2) * stride + (col_off + 1)] = v.x;
    m[(row_off + 2) * stride + (col_off + 2)] = 0.0f;
}

void _ekf_fill_mat3(f32* dst, f32 src[3 * 3], u32 row_off, u32 col_off, u32 stride) {
    dst[(row_off + 0) * stride + (col_off + 0)] = src[0];
    dst[(row_off + 0) * stride + (col_off + 1)] = src[1];
    dst[(row_off + 0) * stride + (col_off + 2)] = src[2];

    dst[(row_off + 1) * stride + (col_off + 0)] = src[3];
    dst[(row_off + 1) * stride + (col_off + 1)] = src[4];
    dst[(row_off + 1) * stride + (col_off + 2)] = src[5];

    dst[(row_off + 2) * stride + (col_off + 0)] = src[6];
    dst[(row_off + 2) * stride + (col_off + 1)] = src[7];
    dst[(row_off + 2) * stride + (col_off + 2)] = src[8];
}

void _ekf_linearize_state_transition(
    extended_kalman_filter* ekf,
    // Output state transition matrix
    f32 F[EKF_STATE_DIM * EKF_STATE_DIM], 
    vec3f gyro_meas_radps,
    vec3f accel_meas_fps2
) {
}

void ekf_predict(
    extended_kalman_filter* ekf, ekf_control_input* control, f32 dt
) {
    vec3f gyro_meas_radps = vec3f_sub(
        control->gyro_radps,
        ekf->nominal_state.gyro_bias_radps
    );

    vec3f accel_meas_fps2 = vec3f_sub(
        control->accel_fps2,
        ekf->nominal_state.accel_bias_fps2
    );

    vec3f world_accel_fps2 = vec3f_add(
        // Accelerometers measure a constant -1g for gravity
        (vec3f){ 0.0f, 0.0f, 32.174f },
        quatf_rot_vec3f(
            ekf->nominal_state.attitude,
            accel_meas_fps2
        )
    );

    // Updating attitude
    // q_k|k-1 = q_k-1|k-1 + 0.5 * dt * q_k-1|k-1 * (0, omega)
    quatf attitude_diff = quatf_mul(
        ekf->nominal_state.attitude,
        (quatf){
            .w = 0.0f,
            .x = 0.5f * dt * gyro_meas_radps.x,
            .y = 0.5f * dt * gyro_meas_radps.y,
            .z = 0.5f * dt * gyro_meas_radps.z,
        }
    );
    
    quatf new_attitude = quatf_norm(
        quatf_add(ekf->nominal_state.attitude, attitude_diff)
    );

    // Updating position and velocity
    vec3f new_vel_fps = vec3f_add(
        ekf->nominal_state.vel_fps,
        vec3f_scale(world_accel_fps2, dt)
    );

    vec3f new_pos_ft = vec3f_add(
        ekf->nominal_state.pos_ft,
        vec3f_add(
            vec3f_scale(ekf->nominal_state.vel_fps, dt),
            vec3f_scale(world_accel_fps2, 0.5f * dt * dt)
        )
    );

    // In the prediction step, the accelerometer and gyro bias are unchanged

    // Next, update the state's covariance given a linearization of the state
    // transition, control model, and process covariance

    // TODO: covariance update

    ekf->nominal_state.attitude = new_attitude;
    ekf->nominal_state.vel_fps = new_vel_fps;
    ekf->nominal_state.pos_ft = new_pos_ft;
}

void ekf_update(extended_kalman_filter* ekf, ekf_measure* measure) {
}
