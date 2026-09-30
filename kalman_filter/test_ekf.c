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

