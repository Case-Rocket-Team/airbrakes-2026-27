
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
    vec3f gyro_radps,
    vec3f accel_fps2,
    f32 dt
) {
    memset(F, 0, sizeof(f32) * EKF_STATE_DIM * EKF_STATE_DIM);

    // Filling in gryo update for small angle error
    gyro_radps.x *= -1;
    gyro_radps.y *= -1;
    gyro_radps.z *= -1;
    _ekf_fill_skew3(F, gyro_radps, 0, 0, EKF_STATE_DIM);

    // Filling in attitude based error props
    {
        f32 R[3 * 3];
        quatf_to_mat3(ekf->nominal_state.attitude, R);
        for (u32 i = 0; i < 9; i++) { R[i] *= -1.0f; }

        _ekf_fill_mat3(F, R, 1 * 3, 4 * 3, EKF_STATE_DIM);

        f32 f_b[3 * 3];
        _ekf_fill_skew3(f_b, accel_fps2, 0, 0, 3);

        f32 res[3 * 3] = { 0 };
        matmul(
            false, false,
            3, 3, 3,
            1.0f, R, f_b,
            0.0f, res
        );

        _ekf_fill_mat3(F, res, 1  * 3, 0, EKF_STATE_DIM);
    }

    // Filling in I and -I error props
    {
        f32 I[3 * 3] = {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
        };

        _ekf_fill_mat3(F, I, 2 * 3, 1 * 3, EKF_STATE_DIM);

        I[0 * 3 + 0] *= -1;
        I[1 * 3 + 1] *= -1;
        I[2 * 3 + 2] *= -1;

        _ekf_fill_mat3(F, I, 0, 3 * 3, EKF_STATE_DIM);
    }

    // Right here, F stores the jacobian of the error state transition. To get
    // to the full state transition, you need to multiply by dt and add I
    for (u32 i = 0; i < EKF_STATE_DIM; i++) {
        for (u32 j = 0; j < EKF_STATE_DIM; j++) {
            F[i * EKF_STATE_DIM + j] *= dt;

            if (i == j) {
                F[i * EKF_STATE_DIM + j] += 1.0f;
            }
        }
    }
}

// Given a float `src`, fills in a 3x3 region of `dst` with src along the
// diagonal. Only the diagonal entries are actually set
void _ekf_fill_diag3(f32* dst, f32 src, u32 row_off, u32 col_off, u32 stride) {
    dst[(row_off + 0) * stride + (col_off + 0)] = src;
    dst[(row_off + 1) * stride + (col_off + 1)] = src;
    dst[(row_off + 2) * stride + (col_off + 2)] = src;
}

void _ekf_build_process_covariance(
    extended_kalman_filter* ekf,
    f32 Q[EKF_STATE_DIM * EKF_STATE_DIM],
    f32 dt
) {
    memset(Q, 0, sizeof(f32) * EKF_STATE_DIM * EKF_STATE_DIM);

    f32 dt2_2 = dt * dt;
    f32 dt3_3 = dt2_2 * dt;
    f32 dt4_8 = dt2_2 * dt2_2;
    f32 dt5_20 = dt3_3 * dt2_2;

    dt2_2 /= 2.0f;
    dt3_3 /= 3.0f;
    f32 dt3_6 = dt3_3 / 2.0f;
    dt4_8 /= 8.0f;
    dt5_20 /= 20.0f;

    #define _EKF_FILLD(s, r, c) _ekf_fill_diag3(Q, (s), (r) * 3, (c) * 3, EKF_STATE_DIM)

    _EKF_FILLD(ekf->gyro_var_rad2ps2 * dt, 0, 0);
    _EKF_FILLD(-ekf->gyro_bias_var_rad2ps2 * dt2_2, 0, 1);
    _EKF_FILLD(ekf->accel_var_f2ps4 * dt + ekf->accel_bias_var_f2ps4 * dt3_3, 1, 1);
    _EKF_FILLD(ekf->accel_var_f2ps4 * dt2_2 + ekf->accel_bias_var_f2ps4 * dt4_8, 1, 2);
    _EKF_FILLD(-ekf->accel_bias_var_f2ps4 * dt2_2, 1, 4);
    _EKF_FILLD(ekf->accel_var_f2ps4 * dt2_2 + ekf->accel_bias_var_f2ps4 * dt4_8, 2, 1);
    _EKF_FILLD(ekf->accel_var_f2ps4 * dt3_3 + ekf->accel_bias_var_f2ps4 * dt5_20, 2, 2);
    _EKF_FILLD(-ekf->accel_bias_var_f2ps4 * dt2_2, 2, 4);
    _EKF_FILLD(-ekf->gyro_bias_var_rad2ps2 * dt2_2, 3, 0);
    _EKF_FILLD(ekf->gyro_bias_var_rad2ps2 * dt, 3, 3);
    _EKF_FILLD(-ekf->accel_bias_var_f2ps4 * dt2_2, 4, 1);
    _EKF_FILLD(-ekf->accel_bias_var_f2ps4 * dt3_6, 4, 2);
    _EKF_FILLD(ekf->accel_bias_var_f2ps4 * dt, 4, 4);
    _EKF_FILLD(ekf->magn_bias_var_gauss2 * dt, 5, 5);

    #undef _EKF_FILLD
}

void ekf_predict(
    extended_kalman_filter* ekf, ekf_control_input* control, f32 dt
) {
    vec3f gyro_radps = vec3f_sub(
        control->gyro_radps,
        ekf->nominal_state.gyro_bias_radps
    );

    vec3f accel_fps2 = vec3f_sub(
        control->accel_fps2,
        ekf->nominal_state.accel_bias_fps2
    );

    vec3f world_accel_fps2 = vec3f_add(
        // Accelerometers measure a constant -1g for gravity
        (vec3f){ 0.0f, 0.0f, 32.174f },
        quatf_rot_vec3f(
            ekf->nominal_state.attitude,
            accel_fps2
        )
    );

    // Updating attitude
    // q_k|k-1 = q_k-1|k-1 + 0.5 * dt * q_k-1|k-1 * (0, omega)
    quatf attitude_diff = quatf_mul(
        ekf->nominal_state.attitude,
        (quatf){
            .w = 0.0f,
            .x = 0.5f * dt * gyro_radps.x,
            .y = 0.5f * dt * gyro_radps.y,
            .z = 0.5f * dt * gyro_radps.z,
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

    // In the prediction step, the biases are unchanged

    // Next, update the state's covariance given a linearization of the state
    // transition, control model, and process covariance

    // P_n+1|n = F * P_n|n * F^T + Q
    {
        // Also called F
        f32 state_transition[EKF_STATE_DIM * EKF_STATE_DIM];
        _ekf_linearize_state_transition(
            ekf, state_transition, gyro_radps, accel_fps2, dt
        );

        // Intermediate matrix equal to F * P_n|n
        f32 FP[EKF_STATE_DIM * EKF_STATE_DIM] = { 0 };

        matmul(
            false, false,
            EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM,
            1.0f, state_transition, ekf->state_covar,
            0.0f, FP
        );

        // Building process covariance directly into the state covariance to 
        // avoid an additional temporary variable. Also, P has already been 
        // consumed by the previous operation
        _ekf_build_process_covariance(ekf, ekf->state_covar, dt);

        matmul(
            false, true,
            EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM,
            1.0f, FP, state_transition,
            1.0f, ekf->state_covar
        );
    }

    ekf->nominal_state.attitude = new_attitude;
    ekf->nominal_state.vel_fps = new_vel_fps;
    ekf->nominal_state.pos_ft = new_pos_ft;
}

void ekf_update(extended_kalman_filter* ekf, ekf_measure* measure) {
    // Constructing observation model (commonly denoted H)
    f32 observation_model[EKF_MEASURE_DIM * EKF_STATE_DIM] = { 0 };
    {
        quatf world_to_body = (quatf){
            .w = ekf->nominal_state.attitude.w,
            .x = -ekf->nominal_state.attitude.x,
            .y = -ekf->nominal_state.attitude.y,
            .z = -ekf->nominal_state.attitude.z,
        };

        // TODO: separate baro and magn measure and throw out north vector of
        // length 0
        vec3f body_magn_north = quatf_rot_vec3f(
            world_to_body, measure->magn_north_gauss
        );

        // For attitude
        _ekf_fill_skew3(observation_model, body_magn_north, 1, 0, EKF_STATE_DIM);

        // For magnetometer bias
        observation_model[1 * EKF_STATE_DIM + 15] = 1.0f;
        observation_model[2 * EKF_STATE_DIM + 16] = 1.0f;
        observation_model[3 * EKF_STATE_DIM + 17] = 1.0f;

        // For altitude
        observation_model[0 * EKF_STATE_DIM + 8] = 1.0f;
    }

    // Forming Kalman gain
    f32 kalman_gain[EKF_STATE_DIM * EKF_MEASURE_DIM] = { 0 };
    {
        f32 innovation_covariance_inv[EKF_MEASURE_DIM * EKF_MEASURE_DIM] = { 0 };

        // Initialize with identity
        for (u32 i = 0; i < EKF_MEASURE_DIM; i++) {
            innovation_covariance_inv[i + i * EKF_MEASURE_DIM] = 1.0f;
        }

        // Stores P * observation_model^T
        // This is used to get the innovation covariance as well as the kalman
        // gain
        f32 PH_T[EKF_STATE_DIM * EKF_MEASURE_DIM] = { 0 };

        matmul(
            false, true,
            EKF_STATE_DIM, EKF_MEASURE_DIM, EKF_STATE_DIM,
            1.0f, ekf->state_covar, observation_model,
            0.0f, PH_T
        );

        // S = HPH^T + R
        // Where R is the measurement covariance
        f32 innovation_covariance[EKF_MEASURE_DIM * EKF_MEASURE_DIM];

        // Initializing with R
        memcpy(
            innovation_covariance, ekf->measure_covar,
            sizeof(innovation_covariance)
        );

        // R += HPH^T
        matmul(
            false, false,
            EKF_MEASURE_DIM, EKF_MEASURE_DIM, EKF_STATE_DIM,
            1.0f, observation_model, PH_T,
            1.0f, innovation_covariance
        );

        // Invertting S
        linear_solve(
            EKF_MEASURE_DIM, EKF_MEASURE_DIM,
            innovation_covariance, innovation_covariance_inv
        );

        // K = PH^T * S^-1
        matmul(
            false, false,
            EKF_STATE_DIM, EKF_MEASURE_DIM, EKF_MEASURE_DIM,
            1.0f, PH_T, innovation_covariance_inv,
            0.0f, kalman_gain
        );
    }

    // Updating covariance according to
    // P_n|n = (I - KH) * P_n|n-1 * (I - KH)^T + KRK^T
    {
        // Stores (I - KH)
        f32 cov_factor[EKF_STATE_DIM * EKF_STATE_DIM] = { 0 };

        // Initializing with identity
        for (u32 i = 0; i < EKF_STATE_DIM; i++) {
            cov_factor[i + i * EKF_STATE_DIM] = 1.0f;
        }

        // I - KH
        matmul(
            false, false,
            EKF_STATE_DIM, EKF_STATE_DIM, EKF_MEASURE_DIM,
            -1.0f, kalman_gain, observation_model,
            1.0f, cov_factor
        );

        // Temporarily stores (I - KH) * P_n|n-1 and later KR
        f32 leftmul[EKF_STATE_DIM * MAX(EKF_MEASURE_DIM, EKF_STATE_DIM)] = { 0 };

        // (I - KH) * P_n|n-1
        matmul(
            false, false,
            EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM,
            1.0f, cov_factor, ekf->state_covar,
            0.0f, leftmul
        );

        // P_n|n = (I - KH) * P_n|n-1 * (I - KH)^T
        // (Just the first half of the covariaince update)
        matmul(
            false, true,
            EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM,
            1.0f, leftmul, cov_factor,
            0.0f, ekf->state_covar
        );

        // K*R
        matmul(
            false, false,
            EKF_STATE_DIM, EKF_MEASURE_DIM, EKF_MEASURE_DIM,
            1.0f, kalman_gain, ekf->measure_covar,
            0.0f, leftmul
        );

        // P_n|n += KRK^T
        matmul(
            false, true,
            EKF_STATE_DIM, EKF_STATE_DIM, EKF_MEASURE_DIM,
            1.0f, leftmul, kalman_gain,
            1.0f, ekf->state_covar
        );
    }

    // Updating nominal state
    {
        ekf_measure predicted_measure = {
            .altitude_ft = ekf->nominal_state.pos_ft.z,
            .magn_north_gauss = vec3f_add(
                quatf_rot_vec3f(
                    ekf->nominal_state.attitude,
                    ekf->world_magn_north_guass
                ),
                ekf->nominal_state.magn_bias_gauss
            )
        };

        // y = (z - h(x))
        f32 innovation_vec[EKF_MEASURE_DIM];
        for (u32 i = 0; i < EKF_MEASURE_DIM; i++) {
            innovation_vec[i] = measure->v[i] - predicted_measure.v[i];
        }

        ekf_err_state err = { 0 };

        // e = Ky
        matmul(
            false, false,
            EKF_STATE_DIM, 1, EKF_MEASURE_DIM,
            1.0f, kalman_gain, innovation_vec,
            0.0f, err.v
        );

        // Inject error correction back into nominal state
        quatf rot_err = {
            .w = 1.0f,
            .x = 0.5f * err.small_angle_rad.x,
            .y = 0.5f * err.small_angle_rad.y,
            .z = 0.5f * err.small_angle_rad.z,
        };

        ekf_nominal_state nominal = ekf->nominal_state;

        ekf->nominal_state = (ekf_nominal_state){
            .attitude = quatf_norm(quatf_mul(nominal.attitude, rot_err)),

            .pos_ft = vec3f_add(nominal.pos_ft, err.pos_ft),
            .vel_fps = vec3f_add(nominal.vel_fps, err.vel_fps),

            .gyro_bias_radps = vec3f_add(
                nominal.gyro_bias_radps, err.gyro_bias_radps
             ),

            .accel_bias_fps2 = vec3f_add(
                nominal.accel_bias_fps2, err.accel_bias_fps2
             ),

            .magn_bias_gauss = vec3f_add(
                nominal.magn_bias_gauss, err.magn_bias_gauss
            ),
        };
    }
}
