#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>

#include "base/base.h"
#include "utils/prng.h"
#include "extended_kalman_filter.h"

#include "base/base.c"
#include "utils/prng.c"
#include "extended_kalman_filter.c"

typedef struct {
    u32 len;

    f32* time_s;
    quatf* attitude;
    vec3f* pos_ft;
    vec3f* vel_fps;
    vec3f* gyro_radps;
    vec3f* accel_fps2;
} ork_data_6dof;

typedef struct {
    f32 mean;
    f32 min;
    f32 median;
    f32 max;
} arr_stats;

f32 err_fn(f32 a, f32 b);

// Note: this modifies the array
arr_stats calc_stats(f32* nums, u32 n);
void print_stats(arr_stats stats);

void read_ork_6dof(const char* path, ork_data_6dof* ork);

vec3f vec3f_perturb(prng* rng, vec3f v, f32 var);

int main(void) {
    ork_data_6dof ork = { 0 };

    // This file is created by utils/ork_process.py
    read_ork_6dof("ork_processed_6dof.bin", &ork);

    prng rng = { 0 };
    prng_seed(&rng, 1, 1);

    vec3f world_north = { 0.0f, 1.0f, 0.0f };

    // Note(Ian) As best I could, these values are relatively accurate to
    // our sensors and their operating conditions. These values will 
    // require more care and tuning for the final version
    ekf_settings settings = {
        .accel_var_f2ps4 = 0.011196358441216f,
        .accel_bias_var_f2ps4 = 1e-4f,

        .gyro_var_rad2ps2 = 1.09662e-6f,
        .gyro_bias_var_rad2ps2 = 1e-5f,

        .baro_var_ft2 = 100.0f,

        .world_magn_north_guass = world_north,
        .magn_covar_gauss2 = {
            3.2e-3f * 3.2e-3f, 0.0f, 0.0f,
            0.0f, 3.2e-3f * 3.2e-3f, 0.0f,
            0.0f, 0.0f, 4.1e-3f * 4.1e-3f,
        },
        .magn_bias_var_gauss2 = 1e-5f,

        .gnss_covar_ft2 = {
            18.0f, 0.0f, 0.0f,
            0.0f, 18.0f, 0.0f,
            0.0f, 0.0f, 200.0f,
        },
    };

    f32* att_similarity = calloc(ork.len, sizeof(f32));
    f32* pos_err_x = calloc(ork.len, sizeof(f32)); 
    f32* pos_err_y = calloc(ork.len, sizeof(f32));
    f32* pos_err_z = calloc(ork.len, sizeof(f32));
    f32* vel_err_x = calloc(ork.len, sizeof(f32));
    f32* vel_err_y = calloc(ork.len, sizeof(f32));
    f32* vel_err_z = calloc(ork.len, sizeof(f32));

    extended_kalman_filter ekf;
    ekf_init(&ekf, &settings, 0);

    for (u32 i = 0; i < ork.len; i++) {
        u32 ts = (u32)(ork.time_s[i] * 1e6f);

        vec3f accel_fps2 = vec3f_perturb(
            &rng, ork.accel_fps2[i], settings.accel_var_f2ps4
        );
        vec3f gyro_radps = vec3f_perturb(
            &rng, ork.gyro_radps[i], settings.gyro_var_rad2ps2
        );

        ekf_inject_imu(&ekf, accel_fps2, gyro_radps, ts);

        f32 baro_altitude_ft = ork.pos_ft[i].z +
            prng_std_norm(&rng) * sqrtf(settings.baro_var_ft2);

        ekf_inject_baro(&ekf, baro_altitude_ft, ts);

        quatf world_to_body = (quatf){
            .w = ork.attitude[i].w,
            .x = -ork.attitude[i].x,
            .y = -ork.attitude[i].y,
            .z = -ork.attitude[i].z,
        };

        vec3f magn_gauss = vec3f_perturb(
            &rng,
            quatf_rot_vec3f(world_to_body, world_north),
            5e-3f * 5e-3f
        );

        ekf_inject_magn(&ekf, magn_gauss, ts);

        if ((i % 20) == 0) {
            vec3f gnss_ft = vec3f_add(
                ork.pos_ft[i],
                (vec3f){
                    sqrtf(ekf.settings.gnss_covar_ft2[0 * 3 + 0]) * prng_std_norm(&rng),
                    sqrtf(ekf.settings.gnss_covar_ft2[1 * 3 + 1]) * prng_std_norm(&rng),
                    sqrtf(ekf.settings.gnss_covar_ft2[2 * 3 + 2]) * prng_std_norm(&rng),
                }
            );

            ekf_inject_gnss(&ekf, gnss_ft, ts);
        }

        att_similarity[i] = acosf(
            quatf_dot(ork.attitude[i], ekf.nominal_state.attitude)
        ) * 180.0f / 3.1415926535f;

        pos_err_x[i] = err_fn(ork.pos_ft[i].x, ekf.nominal_state.pos_ft.x);
        pos_err_y[i] = err_fn(ork.pos_ft[i].y, ekf.nominal_state.pos_ft.y);
        pos_err_z[i] = err_fn(ork.pos_ft[i].z, ekf.nominal_state.pos_ft.z);
        vel_err_x[i] = err_fn(ork.vel_fps[i].x, ekf.nominal_state.vel_fps.x);
        vel_err_y[i] = err_fn(ork.vel_fps[i].y, ekf.nominal_state.vel_fps.y);
        vel_err_z[i] = err_fn(ork.vel_fps[i].z, ekf.nominal_state.vel_fps.z);
    }

    printf("Attitude Stats - ");
    print_stats(calc_stats(att_similarity, ork.len));

    printf("Position Err x - ");
    print_stats(calc_stats(pos_err_x, ork.len));
    printf("Position Err y - ");
    print_stats(calc_stats(pos_err_y, ork.len));
    printf("Position Err z - ");
    print_stats(calc_stats(pos_err_z, ork.len));

    printf("Velocity Err x - ");
    print_stats(calc_stats(vel_err_x, ork.len));
    printf("Velocity Err y - ");
    print_stats(calc_stats(vel_err_y, ork.len));
    printf("Velocity Err z - ");
    print_stats(calc_stats(vel_err_z, ork.len));

    return 0;
}

f32 err_fn(f32 a, f32 b) {
    return ABS(a - b);
}

arr_stats calc_stats(f32* nums, u32 n) {
    arr_stats stats = { 0 };

    f32 sum = 0.0f;
    for (u32 i = 0; i < n; i++) {
        sum += nums[i];
    }

    stats.mean = sum / (f32)n;

    for (u32 i = 1; i < n; i++) {
        for (u32 j = i; j >= 1 && nums[j] < nums[j-1]; j--) {
            f32 tmp = nums[j];
            nums[j] = nums[j-1];
            nums[j-1] = tmp;
        }
    }

    stats.min = nums[0];
    stats.median = n % 2 == 0 ? 0.5f * (nums[n/2 - 1] + nums[n/2]) : nums[n/2];
    stats.max = nums[n-1];

    return stats;
}

void print_stats(arr_stats stats) {
    printf(
        "{ Mean: %f, Min: %f, Median: %f, Max: %f }\n",
        stats.mean, stats.min, stats.median, stats.max
    );
}

void read_ork_6dof(const char* path, ork_data_6dof* ork) {
    FILE* f = fopen(path, "rb");

    fread(&ork->len, sizeof(u32), 1, f);

    u64 num_f32s = (u64)ork->len * (1 + 4 + 3 * 4);
    f32* data = malloc(sizeof(f32) * num_f32s);
    fread(data, sizeof(f32), num_f32s, f);

    ork->time_s = data;
    ork->attitude   = (quatf*)(data + ork->len * 1);
    ork->pos_ft     = (vec3f*)(data + ork->len * 5);
    ork->vel_fps    = (vec3f*)(data + ork->len * 8);
    ork->gyro_radps = (vec3f*)(data + ork->len * 11);
    ork->accel_fps2 = (vec3f*)(data + ork->len * 14);

    fclose(f);
}

vec3f vec3f_perturb(prng* rng, vec3f v, f32 var) {
    return vec3f_add(
        v, vec3f_scale(
            (vec3f){
                prng_std_norm(rng),
                prng_std_norm(rng),
                prng_std_norm(rng),
            },
            sqrtf(var)
        )
    );
}

