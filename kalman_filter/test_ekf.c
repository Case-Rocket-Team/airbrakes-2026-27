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

f32 err_fn(f32 a, f32 b);

void read_ork_6dof(const char* path, ork_data_6dof* ork);

vec3f vec3f_perturb(prng* rng, vec3f v, f32 var);

void run_ekf_step(
    extended_kalman_filter* ekf, FILE* out_file, prng* rng,
    u32 ts, quatf attitude, vec3f local_accel_fps2,
    vec3f ref_gyro_radps, vec3f pos_ft, vec3f vel_fps,
    vec3f accel_bias_fps2, vec3f gyro_bias_radps, 
    vec3f magn_bias_gauss, b32 include_gnss
);

f32 get_arg_f32(i32 argc, char** argv, const char* arg_name, f32 def) {
    for (i32 i = 0; i < argc; i++) {
        if (strcmp(argv[i], arg_name) == 0 && i + 1 < argc) {
            return strtof(argv[i+1], NULL);
        }
    }

    return def;
}

const char* get_arg_cstr(
    i32 argc, char** argv,
    const char* arg_name, const char* def
) {
    for (i32 i = 0; i < argc; i++) {
        if (strcmp(argv[i], arg_name) == 0 && i + 1 < argc) {
            return argv[i+1];
        }
    }

    return def;
}

int main(int argc, char** argv) {
    printf(
        "Test EKF Usage\n\n"
        "test_ekf "
        "--rng-seed [seed] "
        "--pad-time [seconds (defaults to 20)] "
        "--sensor-bias [zero|fixed] "
        "\n\n"
    );

    ork_data_6dof ork = { 0 };

    // This file is created by utils/ork_process.py
    read_ork_6dof("ork_processed_6dof.bin", &ork);

    prng rng = { 0 };
    prng_seed(&rng, (u32)get_arg_f32(argc, argv, "--rng-seed", 1.0f), 1);

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
    ekf_init(
        &ekf, &settings, 
        quatf_rot_vec3f(quatf_conj(ork.attitude[0]), (vec3f){ 0.0f, 0.0f, -1.0f }),
        quatf_rot_vec3f(quatf_conj(ork.attitude[0]), world_north),
        0
    );

    const char* bias_setting = get_arg_cstr(argc, argv, "--sensor-bias", "fixed");

    printf("bias setting: %s\n", bias_setting);

    vec3f accel_bias_fps2 = { 0 };
    vec3f gyro_bias_radps = { 0 };
    vec3f magn_bias_gauss = { 0 };

    if (strcmp(bias_setting, "fixed") == 0) {
        accel_bias_fps2 = (vec3f){
            0.1f * prng_rand_f32(&rng),
            0.1f * prng_rand_f32(&rng),
            0.1f * prng_rand_f32(&rng),
        };

        gyro_bias_radps = (vec3f){
            0.1f * prng_rand_f32(&rng),
            0.1f * prng_rand_f32(&rng),
            0.1f * prng_rand_f32(&rng),
        };

        magn_bias_gauss = (vec3f){
            0.1f * prng_rand_f32(&rng),
            0.1f * prng_rand_f32(&rng),
            0.1f * prng_rand_f32(&rng),
        };
    }

    f32 onpad_time_s = get_arg_f32(argc, argv, "--pad-time", 20.0f);
    u32 onpad_steps = (u32)(onpad_time_s / 0.01f);
    for (u32 i = 0; i < onpad_steps; i++) {
        u32 ts = (u32)((f32)i * 0.01f * 1e6f);

        quatf world_to_body = quatf_conj(ork.attitude[0]);

        run_ekf_step(
            &ekf, out_file, &rng, ts,
            ork.attitude[0], quatf_rot_vec3f(
                world_to_body, (vec3f){ 0, 0, 32.174f }
            ), (vec3f){ 0 }, ork.pos_ft[0], (vec3f){ 0 },
            accel_bias_fps2, gyro_bias_radps, magn_bias_gauss,
            (i % 20) == 0
        );
    }

    for (u32 i = 0; i < ork.len; i++) {
        u32 ts = (u32)((ork.time_s[i] + onpad_time_s) * 1e6f);

        run_ekf_step(
            &ekf, out_file, &rng, ts,
            ork.attitude[i], ork.accel_fps2[i],
            ork.gyro_radps[i], ork.pos_ft[i], ork.vel_fps[i],
            accel_bias_fps2, gyro_bias_radps, magn_bias_gauss,
            (i % 20) == 0
        );
    }

    fclose(out_file);

    free(ork.time_s);

    return 0;
}

void run_ekf_step(
    extended_kalman_filter* ekf, FILE* out_file, prng* rng,
    u32 ts, quatf attitude, vec3f local_accel_fps2,
    vec3f ref_gyro_radps, vec3f pos_ft, vec3f vel_fps,
    vec3f accel_bias_fps2, vec3f gyro_bias_radps, 
    vec3f magn_bias_gauss, b32 include_gnss
) {
    quatf world_to_body = quatf_conj(attitude);

    vec3f accel_fps2 = vec3f_add(
        vec3f_perturb(
            rng, accel_bias_fps2, ekf->settings.accel_bias_var_f2ps4
        ),
        vec3f_perturb(
            rng, local_accel_fps2, ekf->settings.accel_var_f2ps4
        )
    );

    vec3f gyro_radps = vec3f_add(
        vec3f_perturb(
            rng, gyro_bias_radps, ekf->settings.gyro_bias_var_rad2ps2
        ),
        vec3f_perturb(
            rng, ref_gyro_radps, ekf->settings.gyro_var_rad2ps2
        )
    );

    ekf_inject_imu(
        ekf, accel_fps2, gyro_radps, ts
    );

    f32 baro_altitude_ft = pos_ft.z +
        prng_std_norm(rng) * sqrtf(ekf->settings.baro_var_ft2);

    ekf_inject_baro(ekf, baro_altitude_ft, ts);

    vec3f magn_gauss = vec3f_add(
        vec3f_perturb(
            rng, magn_bias_gauss, ekf->settings.magn_bias_var_gauss2
        ),
        vec3f_perturb(
            rng,
            quatf_rot_vec3f(world_to_body, ekf->world_magn_north_guass),
            5e-3f * 5e-3f
        )
    );

    ekf_inject_magn(ekf, magn_gauss, ts);

    if (include_gnss) {
        vec3f gnss_ft = vec3f_add(
            pos_ft,
            (vec3f){
                sqrtf(ekf->settings.gnss_covar_ft2[0 * 3 + 0]) * prng_std_norm(rng),
                sqrtf(ekf->settings.gnss_covar_ft2[1 * 3 + 1]) * prng_std_norm(rng),
                sqrtf(ekf->settings.gnss_covar_ft2[2 * 3 + 2]) * prng_std_norm(rng),
            }
        );

        ekf_inject_gnss(ekf, gnss_ft, ts);
    }

    f32 att_dot = quatf_dot(attitude, ekf->nominal_state.attitude);
    f32 att_similarity = 2.0f * acosf(
        CLAMP(att_dot, -1.0f, 1.0f)
    ) * 180.0f / 3.1415926535f;

    f32 pos_err_x = err_fn(pos_ft.x, ekf->nominal_state.pos_ft.x);
    f32 pos_err_y = err_fn(pos_ft.y, ekf->nominal_state.pos_ft.y);
    f32 pos_err_z = err_fn(pos_ft.z, ekf->nominal_state.pos_ft.z);
    f32 vel_err_x = err_fn(vel_fps.x, ekf->nominal_state.vel_fps.x);
    f32 vel_err_y = err_fn(vel_fps.y, ekf->nominal_state.vel_fps.y);
    f32 vel_err_z = err_fn(vel_fps.z, ekf->nominal_state.vel_fps.z);

    f32 accel_bias_err_x = err_fn(accel_bias_fps2.x, ekf->nominal_state.accel_bias_fps2.x);
    f32 accel_bias_err_y = err_fn(accel_bias_fps2.y, ekf->nominal_state.accel_bias_fps2.y);
    f32 accel_bias_err_z = err_fn(accel_bias_fps2.z, ekf->nominal_state.accel_bias_fps2.z);

    f32 gyro_bias_err_x = err_fn(gyro_bias_radps.x, ekf->nominal_state.gyro_bias_radps.x);
    f32 gyro_bias_err_y = err_fn(gyro_bias_radps.y, ekf->nominal_state.gyro_bias_radps.y);
    f32 gyro_bias_err_z = err_fn(gyro_bias_radps.z, ekf->nominal_state.gyro_bias_radps.z);

    f32 magn_bias_err_x = err_fn(magn_bias_gauss.x, ekf->nominal_state.magn_bias_gauss.x);
    f32 magn_bias_err_y = err_fn(magn_bias_gauss.y, ekf->nominal_state.magn_bias_gauss.y);
    f32 magn_bias_err_z = err_fn(magn_bias_gauss.z, ekf->nominal_state.magn_bias_gauss.z);

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

f32 err_fn(f32 a, f32 b) {
    return a - b;
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

