
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

void _ekf_fill_quatmat(f32* m, quatf q, u32 row_off, u32 col_off, u32 stride) {
    f32 r_mat[9];
    quatf_to_mat3(q, r_mat);

    m[(row_off + 0) * stride + (col_off + 0)] = r_mat[0];
    m[(row_off + 0) * stride + (col_off + 1)] = r_mat[1];
    m[(row_off + 0) * stride + (col_off + 2)] = r_mat[2];

    m[(row_off + 1) * stride + (col_off + 0)] = r_mat[3];
    m[(row_off + 1) * stride + (col_off + 1)] = r_mat[4];
    m[(row_off + 1) * stride + (col_off + 2)] = r_mat[5];

    m[(row_off + 2) * stride + (col_off + 0)] = r_mat[6];
    m[(row_off + 2) * stride + (col_off + 1)] = r_mat[7];
    m[(row_off + 2) * stride + (col_off + 2)] = r_mat[8];
}

void ekf_predict(
    extended_kalman_filter* ekf, ekf_control_input* control, f32 dt
) {
    // First, update nominal state given control input
    quatf world_to_body = ekf->nominal_state.attitude;
    quatf body_to_world = (quatf){
        .w = world_to_body.w,
        .x = -world_to_body.x,
        .y = -world_to_body.y,
        .z = -world_to_body.z,
    };

    vec3f gyro_radps = vec3f_sub(
        control->gyro_radps,
        ekf->nominal_state.gyro_bias_radps
    );

    vec3f world_accel_fps2 = vec3f_add(
        // Accelerometers measure a constant -1g for gravity
        (vec3f){ 0.0f, 0.0f, 32.174f },
        quatf_rot_vec3f(
            body_to_world,
            vec3f_sub(
                control->accel_fps2,
                ekf->nominal_state.accel_bias_fps2
            )
        )
    );

    // Updating attitude
    // q_k|k-1 = q_k-1|k-1 + 0.5 * dt * (0, omega) * q_k-1|k-1
    quatf attitude_diff = quatf_mul(
        (quatf){
            .w = 0.0f,
            .x = 0.5f * dt * gyro_radps.x,
            .y = 0.5f * dt * gyro_radps.y,
            .z = 0.5f * dt * gyro_radps.z,
        },
        ekf->nominal_state.attitude
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
