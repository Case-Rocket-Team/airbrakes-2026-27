
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
}

void ekf_update(extended_kalman_filter* ekf, ekf_measure* measure) {
}
