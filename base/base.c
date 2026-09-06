
#include "base.h"

vec2f vec2f_add(vec2f a, vec2f b) {
    return (vec2f){ a.x + b.x, a.y + b.y };
}

vec2f vec2f_sub(vec2f a, vec2f b) {
    return (vec2f){ a.x - b.x, a.y - b.y };
}

vec2f vec2f_comp_mul(vec2f a, vec2f b) {
    return (vec2f){ a.x * b.x, a.y * b.y };
}

vec2f vec2f_comp_div(vec2f a, vec2f b) {
    return (vec2f){ a.x / b.x, a.y / b.y };
}

vec2f vec2f_scale(vec2f v, f32 s) {
    return (vec2f){ v.x * s, v.y * s };
}

vec2f vec2f_perp(vec2f v) {
    return (vec2f){ -v.y, v.x };
}

f32 vec2f_cross(vec2f a, vec2f b) {
    return a.x * b.y - a.y * b.x;
}

f32 vec2f_dot(vec2f a, vec2f b) {
    return a.x * b.x + a.y * b.y;
}

f32 vec2f_sqr_dist(vec2f a, vec2f b) {
    return (a.x - b.x) * (a.x - b.x) +
        (a.y - b.y) * (a.y - b.y);
}

f32 vec2f_dist(vec2f a, vec2f b) {
    return sqrtf((a.x - b.x) * (a.x - b.x) +
        (a.y - b.y) * (a.y - b.y));
}

b8 vec2f_eq(vec2f a, vec2f b) {
    return a.x == b.x && a.y == b.y;
}

f32 vec2f_sqr_len(vec2f v) {
    return v.x * v.x + v.y * v.y;
}

f32 vec2f_len(vec2f v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

vec2f vec2f_norm(vec2f v) {
    f32 len = sqrtf(v.x * v.x + v.y * v.y);

    if (ABS(len) > 1e-8f) {
        return (vec2f){ v.x / len, v.y / len };
    }

    return (vec2f){ 1, 0 };
}

vec3f vec3f_add(vec3f a, vec3f b) {
    return (vec3f){ a.x + b.x, a.y + b.y, a.z + b.z };
}

vec3f vec3f_sub(vec3f a, vec3f b) {
    return (vec3f){ a.x - b.x, a.y - b.y, a.z - b.z };
}

vec3f vec3f_comp_mul(vec3f a, vec3f b) {
    return (vec3f){ a.x * b.x, a.y * b.y, a.z * b.z };
}

vec3f vec3f_comp_div(vec3f a, vec3f b) {
    return (vec3f){ a.x / b.x, a.y / b.y, a.z / b.z };
}

vec3f vec3f_scale(vec3f v, f32 s) {
    return (vec3f){ v.x * s, v.y * s, v.z * s };
}

vec3f vec3f_cross(vec3f a, vec3f b) {
    return (vec3f) {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

f32 vec3f_dot(vec3f a, vec3f b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

f32 vec3f_sqr_dist(vec3f a, vec3f b) {
    return (a.x - b.x) * (a.x - b.x) + 
        (a.y - b.y) * (a.y - b.y) + 
        (a.z - b.z) * (a.z - b.z);
}

f32 vec3f_dist(vec3f a, vec3f b) {
    return sqrtf((a.x - b.x) * (a.x - b.x) + 
        (a.y - b.y) * (a.y - b.y) + 
        (a.z - b.z) * (a.z - b.z));
}

b8 vec3f_eq(vec3f a, vec3f b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

f32 vec3f_sqr_len(vec3f v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

f32 vec3f_len(vec3f v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

vec3f vec3f_norm(vec3f v) {
    f32 len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

    if (ABS(len) > 1e-8f) {
        f32 r = 1.0f / len;
        return (vec3f){ v.x * r, v.y * r, v.z * r };
    }

    return (vec3f){ 1, 0, 0 };
}

void _mm_nn(
    u32 c_rows, u32 c_cols, u32 ap_cols,
    f32 alpha, f32* A, f32* B, f32* C
) {
    for (u32 i = 0; i < c_cols; i++) {
        for (u32 k = 0; k < ap_cols; k++) {
            for (u32 j = 0; j < c_rows; j++) {
                C[i + j * c_cols] += alpha * A[k + j * ap_cols] * B[i + k * c_cols];
            }
        }
    }
}

void _mm_nt(
    u32 c_rows, u32 c_cols, u32 ap_cols,
    f32 alpha, f32* A, f32* B, f32* C
) {
    for (u32 k = 0; k < ap_cols; k++) {
        for (u32 i = 0; i < c_cols; i++) {
            for (u32 j = 0; j < c_rows; j++) {
                C[i + j * c_cols] += alpha * A[k + j * ap_cols] * B[k + i * ap_cols];
            }
        }
    }
}

void _mm_tn(
    u32 c_rows, u32 c_cols, u32 ap_cols,
    f32 alpha, f32* A, f32* B, f32* C
) {
    for (u32 i = 0; i < c_cols; i++) {
        for (u32 j = 0; j < c_rows; j++) {
            for (u32 k = 0; k < ap_cols; k++) {
                C[i + j * c_cols] += alpha * A[j + k * c_rows] * B[i + k * c_cols];
            }
        }
    }
}

void _mm_tt(
    u32 c_rows, u32 c_cols, u32 ap_cols,
    f32 alpha, f32* A, f32* B, f32* C
) {
    for (u32 i = 0; i < c_cols; i++) {
        for (u32 k = 0; k < ap_cols; k++) {
            for (u32 j = 0; j < c_rows; j++) {
                C[i + j * c_cols] += alpha * A[j + k * c_rows] * B[k + i * ap_cols];
            }
        }
    }
}

void matmul(
    b8 transpose_a, b8 transpose_b,
    u32 c_rows, u32 c_cols, u32 ap_cols, 
    f32 alpha, f32* A, f32* B, f32 beta, f32* C
) {
    u32 c_size = c_rows * c_cols;

    if (beta == 0.0f) {
        memset(C, 0, c_size * sizeof(f32));
    } else {
        for (u32 i = 0; i < c_size; i++) {
            C[i] *= beta;
        }
    }

    u32 transpose_bits = (u32)((!!transpose_a << 1) | !!transpose_b);
    switch (transpose_bits) {
        case 0b00: _mm_nn(c_rows, c_cols, ap_cols, alpha, A, B, C); break;
        case 0b01: _mm_nt(c_rows, c_cols, ap_cols, alpha, A, B, C); break;
        case 0b10: _mm_tn(c_rows, c_cols, ap_cols, alpha, A, B, C); break;
        case 0b11: _mm_tt(c_rows, c_cols, ap_cols, alpha, A, B, C); break;
        
        default: break;
    }
}

