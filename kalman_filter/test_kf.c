
#include <stdio.h>

#include "kalman_filter.h"
#include "kalman_filter.c"

int main(void) {
    kalman_filter kf = {
        .state = {
            .vec = {
                .altitude_ft = 0,
                .vertical_vel_fps = 0,
                .accel_bias_fps2 = 0,
            },

            .covariance = {
                100.0f, 0.0f, 0.0f,
                0.0f, 20.0f, 0.0f,
                0.0f, 0.0f, 20.0f,
            }
        }
    };

    printf("Hello World\n");

    return 0;
}

