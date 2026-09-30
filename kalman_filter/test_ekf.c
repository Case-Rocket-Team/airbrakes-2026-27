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

void read_ork_6dof(const char* path, ork_data_6dof* ork);

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

    extended_kalman_filter ekf;
    ekf_init(&ekf, &settings, 0);

    return 0;
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

