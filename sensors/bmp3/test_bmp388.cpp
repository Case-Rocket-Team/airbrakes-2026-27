//   BMP388 SCL -> pin 13 (SCK)
//   BMP388 SDO -> pin 12 (MISO)
//   BMP388 SDA -> pin 11 (MOSI)
//   BMP388 CS  -> pin 10
//   BMP388 INT -> pin 9

#include <Arduino.h>
#include "bmp388.h"

static uint32_t last_ms;
static uint32_t count;
static uint32_t first_ms; // time of first sample, for the running average

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!bmp388_init(10, 9, 2)) {
        Serial.println("BMP388 init failed");
        while (1) {}
    }

    Serial.println("Waiting for interrupts");
    last_ms = millis();
}

void loop() {
    if (!bmp388_data_ready()) {
        return;
    }

    float t, p;
    bmp388_read(&t, &p); // also clears the data-ready flag

    uint32_t now = millis();
    count++;
    if (count == 1) {
        first_ms = now;
    }

    // Average interval since the first sample 
    float avg_ms = (count > 1) ? (float)(now - first_ms) / (count - 1) : 0.0f;

    Serial.print("IRQ #");
    Serial.print(count);
    Serial.print("  dt ");
    Serial.print(now - last_ms);
    Serial.print(" ms  avg ");
    Serial.print(avg_ms, 3);
    Serial.print(" ms  ");
    Serial.print(t);
    Serial.print(" C  ");
    Serial.print(p);
    Serial.println(" Pa");

    last_ms = now;
}
