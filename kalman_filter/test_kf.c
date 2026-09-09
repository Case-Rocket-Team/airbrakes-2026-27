
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>

#include "base/base.h"
#include "kalman_filter.h"

#include "base/base.c"
#include "kalman_filter.c"

typedef struct {
    u32 len;

    f32* time;
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

    printf("%u rows\n", ork.len);

    for (u32 i = 0; i < 10; i++) {
        printf(
            "%7.3f %7.3f, %7.3f %7.3f\n",
            ork.time[i],
            ork.altitude_ft[i],
            ork.vertical_vel_fps[i],
            ork.vertical_accel_fps2[i]
        );
    }
    
    return 0;
}

void read_ork_bin(const char* path, ork_data* ork) {
    FILE* f = fopen(path, "rb");

    fread(&ork->len, sizeof(u32), 1, f);

    f32* data = malloc(sizeof(f32) * ork->len * 4);
    fread(data, sizeof(f32), ork->len * 4, f);

    ork->time = data;
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

