



#include <Arduino.h>
#include "bmp388.h"

#define N 1000
#define pressureLoop 300

static float alt[N];

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!bmp388_init(10, BMP388_NO_INT, BMP388_200HZ_ODR)) {
        Serial.println("BMP388 init failed");
        while (1) {}
    }

    double meanPressure = 0;
    for (int i = 0; i < pressureLoop; i++) {
        while (!bmp388_data_ready()) {}
        float t, p;
        bmp388_read(&t, &p);
        meanPressure += p;
    }

    meanPressure /= pressureLoop;
    
    // for (int i = 0; i < N; i++) {
    //     while (!bmp388_data_ready()) {}
    //     float t, p;
    //     bmp388_read(&t, &p);
    //     alt[i] = bmp388_altitude(p); // ISA altitude above sea level (m)
    // }

    // double mean = 0;
    // for (int i = 0; i < N; i++) {
    //     mean += alt[i];
    // }
    // mean /= N;

    // double var = 0;
    // for (int i = 0; i < N; i++) {
    //     var += (alt[i] - mean) * (alt[i] - mean);
    // }
    // double std = sqrt(var / (N - 1));

    // Serial.print("mean: ");
    // Serial.print(mean, 3);
    // Serial.print(" m  std: ");
    // Serial.print(std, 4);
    // Serial.println(" m");

    for (int i = 0; i < N; i++) {
        while (!bmp388_data_ready()) {}
        float t, p;
        bmp388_read(&t, &p);
        Serial.printf("%.9g\n", bmp388_altitude(p, meanPressure, t)); // 9 sig. digits = full float precision
    }

    
}

void loop() {}
