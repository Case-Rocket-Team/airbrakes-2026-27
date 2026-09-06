
#include <stdio.h>

#include "base/base.h"
#include "kalman_filter.h"

#include "base/base.c"
#include "kalman_filter.c"

void printmat(f32* M, u32 rows, u32 cols) {
    for (u32 j = 0; j < rows; j++) {
        for (u32 i = 0; i < cols; i++) {
            printf("%8.4f, ", M[i + j * cols]);
        }
        printf("\n");
    }
    printf("\n");
}

int main(void) {
    f32 A[] = {
        2, 5, 8,
        5, 2, 2,
        7, 5, 6
    };

    f32 B[] = {
        1, 2,
        3, 4,
        6, 5,
    };

    linear_solve(3, 2, A, B);

    printmat(B, 3, 2);

    return 0;
}

