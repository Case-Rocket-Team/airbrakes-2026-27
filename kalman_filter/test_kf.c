
#include <stdio.h>

#include "base/base.h"
#include "kalman_filter.h"

#include "base/base.c"
#include "kalman_filter.c"

void printmat(f32* M, u32 rows, u32 cols) {
    for (u32 j = 0; j < rows; j++) {
        for (u32 i = 0; i < cols; i++) {
            printf("%3.0f, ", M[i + j * cols]);
        }
        printf("\n");
    }
    printf("\n");
}

int main(void) {
    f32 A[] = {
        1, 2, 3,
        4, 5, 6,
    };

    f32 B[] = {
        2, 3,
        7, 11, 
        17, 19,
    };

    f32 C[9] = { 0 };

    matmul(
        false, false,
        2, 2, 3,
        1.0f, A, B,
        0.0f, C
    );
    printmat(C, 2, 2);

    matmul(
        true, true,
        3, 3, 2,
        1.0f, A, B,
        0.0f, C
    );
    printmat(C, 3, 3);


    return 0;
}

