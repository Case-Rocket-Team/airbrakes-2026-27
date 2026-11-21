#include <Arduino.h>
#include "bmp388.h"

#define N 1000
#define pressureSamples 300

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!bmp388_init(10, BMP388_NO_INT, BMP388_200HZ_ODR)) {
        Serial.println("BMP388 init failed");
        while (1) {}
    }

    // Ground reference: average pressure and temperature while stationary.
    double meanPressure = 0;
    double meanTemp = 0;
    for (int i = 0; i < pressureSamples; i++) {
        while (!bmp388_data_ready()) {}
        bmp388_data data = bmp388_read();
        meanPressure += data.press_pa;
        meanTemp += data.temp_c;
    }

    meanPressure /= pressureSamples;
    meanTemp /= pressureSamples;

    for (int i = 0; i < N; i++) {
        while (!bmp388_data_ready()) {}
        bmp388_data data = bmp388_read();
        Serial.printf("%.9g\n", bmp388_altitude_m(data.press_pa, meanPressure, meanTemp));
    }

    
}

void loop() {}
