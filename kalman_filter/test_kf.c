
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>

#include "base/base.h"
#include "utils/prng.h"
#include "kalman_filter.h"

#include "base/base.c"
#include "utils/prng.c"
#include "kalman_filter.c"

typedef struct {
    u32 len;

    f32* time_s;
    f32* altitude_ft;
    f32* vertical_vel_fps;
    f32* vertical_accel_fps2;
} ork_data;

void printmat(f32* M, u32 rows, u32 cols);
void read_ork_bin(const char* path, ork_data* ork);

int main(void) {
    ork_data ork = { 0 };

    // This file is created by utils/ork_process.py
    read_ork_bin("ork_processed.bin", &ork);

    prng rng = { 0 };
    prng_seed(&rng, 1, 1);

    f32 accel_sdev_fps2 = 0.1058f;
    f32 baro_sdev_ft = 50.0f;

    f32 accel_bias_fps2 = 0.1f;
    f32 accel_bias_sdev_fps2 = 0.01f;

    kalman_filter kf = {
        .state = {
            .vec = { 0 },
            .covariance = {
                1, 1, 0,
                1, 1, 0,
                0, 0, 1,
            }
        },

        .accel_stddev_fps2 = accel_sdev_fps2,
        .accel_bias_stddev_fps2 = accel_bias_sdev_fps2,

        .measure_covariance = {
            baro_sdev_ft
        },
    };

    FILE* out_file = fopen("kf_results.csv", "w");
    fprintf(
        out_file,
        "time_s,"
        "altitude_ft,vertical_vel_fps,accel_bias_fps2,"
        "m_altitude_ft,"
        "kf_altitude_ft,kf_vertical_vel_fps,kf_accel_bias_fps2\n"
    );

    for (u32 i = 1; i < ork.len; i++) {
        f32 dt = ork.time_s[i] - ork.time_s[i-1];

        //accel_bias_fps2 += 0.05f * dt;

        kf_control_input control = {
            .accel_fps2 = ork.vertical_accel_fps2[i-1] +
                accel_sdev_fps2 * prng_std_norm(&rng) +
                accel_bias_fps2 +
                accel_bias_sdev_fps2 * prng_std_norm(&rng)
        };

        kf_measure measure = {
            .altitude_ft = ork.altitude_ft[i] + 
                baro_sdev_ft * prng_std_norm(&rng)
        };

        kf_predict(&kf, &control, dt);
        kf_update(&kf, &measure);

        fprintf(
            out_file,
            "%.10g,"
            "%.10g,%.10g,%.10g,"
            "%.10g,"
            "%.10g,%.10g,%.10g\n",
            ork.time_s[i],
            ork.altitude_ft[i], ork.vertical_vel_fps[i], accel_bias_fps2,
            measure.altitude_ft,
            kf.state.vec.altitude_ft, kf.state.vec.vertical_vel_fps,
            kf.state.vec.accel_bias_fps2
        );
    }

    fclose(out_file);

    return 0;
}

void read_ork_bin(const char* path, ork_data* ork) {
    FILE* f = fopen(path, "rb");

    fread(&ork->len, sizeof(u32), 1, f);

    f32* data = malloc(sizeof(f32) * ork->len * 4);
    fread(data, sizeof(f32), ork->len * 4, f);

    ork->time_s = data;
    ork->altitude_ft = data + ork->len * 1;
    ork->vertical_vel_fps = data + ork->len * 2;
    ork->vertical_accel_fps2 = data + ork->len * 3;

    fclose(f);
}

void printmat(f32* M, u32 rows, u32 cols) {
    for (u32 j = 0; j < rows; j++) {
        for (u32 i = 0; i < cols; i++) {
            printf("%8.4f, ", M[i + j * cols]);
        }
        printf("\n");
    }
    printf("\n");
}

