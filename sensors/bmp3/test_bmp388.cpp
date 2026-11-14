//   BMP388 SCL -> pin 13 (SCK)
//   BMP388 SDO -> pin 12 (MISO)
//   BMP388 SDA -> pin 11 (MOSI)
//   BMP388 CS  -> pin 10
//   BMP388 INT -> pin 9

#include <Arduino.h>
#include "bmp388.h"

// ODR = 200 Hz / 2^odr_sel. 10 = one sample every 5.12 s.
#define SLOW_ODR 10

#define READ_ON_IRQ 1

static uint32_t last_irq_ms;
static uint32_t last_tick_ms;
static uint32_t irq_count;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!bmp388_init(10, 9)) {
        Serial.println("BMP388 init failed");
        while (1) {}
    }
    bmp388_set_odr(SLOW_ODR);

    Serial.println("Waiting for interrupts (expect one every ~5.12 s)...");
    last_irq_ms = last_tick_ms = millis();
}

void loop() {
    uint32_t now = millis();

    if (bmp388_data_ready()) {
        float t = 0, p = 0;
#if READ_ON_IRQ
        bmp388_read(&t, &p);
#endif
        irq_count++;

        Serial.print(">>> INTERRUPT #");
        Serial.print(irq_count);
        Serial.print("  dt ");
        Serial.print(now - last_irq_ms);
        Serial.print(" ms  ");
        Serial.print(t);
        Serial.print(" C  ");
        Serial.print(p);
        Serial.println(" Pa");

        last_irq_ms = last_tick_ms = now;
    }

    if (now - last_tick_ms >= 1000) {
        last_tick_ms += 1000;
        Serial.print(" Doing soooo many other things ");
        Serial.print((now - last_irq_ms) / 1000);
        Serial.println(" s");
    }
}
