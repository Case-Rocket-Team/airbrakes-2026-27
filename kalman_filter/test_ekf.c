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

    FILE* out_file = fopen("ekf_results.csv", "w");
    fprintf(
        out_file,
        "time_us,att_similarity,"
        "pos_err_x,pos_err_y,pos_err_z,"
        "vel_err_x,vel_err_y,vel_err_z,"
        "accel_bias_err_x,accel_bias_err_y,accel_bias_err_z,"
        "gyro_bias_err_x,gyro_bias_err_y,gyro_bias_err_z,"
        "magn_bias_err_x,magn_bias_err_y,magn_bias_err_z,"
        "\n"
    );

    extended_kalman_filter ekf;
    ekf_init(&ekf, &settings, 0);

    vec3f accel_bias_fps2 = {
        prng_rand_f32(&rng),
        prng_rand_f32(&rng),
        prng_rand_f32(&rng),
    };

    vec3f gyro_bias_radps = {
        0.2f * prng_rand_f32(&rng),
        0.2f * prng_rand_f32(&rng),
        0.2f * prng_rand_f32(&rng),
    };

    vec3f magn_bias_gauss = {
        prng_rand_f32(&rng),
        prng_rand_f32(&rng),
        prng_rand_f32(&rng),
    };

    for (u32 i = 0; i < ork.len; i++) {
        u32 ts = (u32)(ork.time_s[i] * 1e6f);

        vec3f accel_fps2 = vec3f_add(
            vec3f_perturb(
                &rng, accel_bias_fps2, settings.accel_bias_var_f2ps4
            ),
            vec3f_perturb(
                &rng, ork.accel_fps2[i], settings.accel_var_f2ps4
            )
        );

        vec3f gyro_radps = vec3f_add(
            vec3f_perturb(
                &rng, gyro_bias_radps, settings.gyro_bias_var_rad2ps2
            ),
            vec3f_perturb(
                &rng, ork.gyro_radps[i], settings.gyro_var_rad2ps2
            )
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

        vec3f magn_gauss = vec3f_add(
            vec3f_perturb(
                &rng, magn_bias_gauss, settings.magn_bias_var_gauss2
            ),
            vec3f_perturb(
                &rng,
                quatf_rot_vec3f(world_to_body, world_north),
                5e-3f * 5e-3f
            )
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

        f32 att_dot = quatf_dot(ork.attitude[i], ekf.nominal_state.attitude);
        f32 att_similarity = acosf(
            CLAMP(att_dot, -1.0f, 1.0f)
        ) * 180.0f / 3.1415926535f;

        f32 pos_err_x = err_fn(ork.pos_ft[i].x, ekf.nominal_state.pos_ft.x);
        f32 pos_err_y = err_fn(ork.pos_ft[i].y, ekf.nominal_state.pos_ft.y);
        f32 pos_err_z = err_fn(ork.pos_ft[i].z, ekf.nominal_state.pos_ft.z);
        f32 vel_err_x = err_fn(ork.vel_fps[i].x, ekf.nominal_state.vel_fps.x);
        f32 vel_err_y = err_fn(ork.vel_fps[i].y, ekf.nominal_state.vel_fps.y);
        f32 vel_err_z = err_fn(ork.vel_fps[i].z, ekf.nominal_state.vel_fps.z);

        f32 accel_bias_err_x = err_fn(accel_bias_fps2.x, ekf.nominal_state.accel_bias_fps2.x);
        f32 accel_bias_err_y = err_fn(accel_bias_fps2.y, ekf.nominal_state.accel_bias_fps2.y);
        f32 accel_bias_err_z = err_fn(accel_bias_fps2.z, ekf.nominal_state.accel_bias_fps2.z);

        f32 gyro_bias_err_x = err_fn(gyro_bias_radps.x, ekf.nominal_state.gyro_bias_radps.x);
        f32 gyro_bias_err_y = err_fn(gyro_bias_radps.y, ekf.nominal_state.gyro_bias_radps.y);
        f32 gyro_bias_err_z = err_fn(gyro_bias_radps.z, ekf.nominal_state.gyro_bias_radps.z);

        f32 magn_bias_err_x = err_fn(magn_bias_gauss.x, ekf.nominal_state.magn_bias_gauss.x);
        f32 magn_bias_err_y = err_fn(magn_bias_gauss.y, ekf.nominal_state.magn_bias_gauss.y);
        f32 magn_bias_err_z = err_fn(magn_bias_gauss.z, ekf.nominal_state.magn_bias_gauss.z);

        fprintf(
            out_file,
            "%u,%.10g,"
            "%.10g,%.10g,%.10g,"
            "%.10g,%.10g,%.10g,"
            "%.10g,%.10g,%.10g,"
            "%.10g,%.10g,%.10g,"
            "%.10g,%.10g,%.10g"
            "\n",
            ts, att_similarity,
            pos_err_x, pos_err_y, pos_err_z,
            vel_err_x, vel_err_y, vel_err_z,
            accel_bias_err_x, accel_bias_err_y, accel_bias_err_z,
            gyro_bias_err_x, gyro_bias_err_y, gyro_bias_err_z,
            magn_bias_err_x, magn_bias_err_y, magn_bias_err_z
        );
    }

    /*printf("Attitude Stats - ");
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
    print_stats(calc_stats(vel_err_z, ork.len));*/

    fclose(out_file);

    free(ork.time_s);

    return 0;
}

f32 err_fn(f32 a, f32 b) {
    return a - b;
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

