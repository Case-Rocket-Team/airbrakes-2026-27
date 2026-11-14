//   BMP388 SCL -> pin 13 (SCK)
//   BMP388 SDO -> pin 12 (MISO)
//   BMP388 SDA -> pin 11 (MOSI)
//   BMP388 CS  -> pin 10

#include <Arduino.h>
#include "bmp388.h"

void setup() {
    Serial.begin(115200);
    bmp388_init(10);
}

void loop() {
    float t, p;
    bmp388_read(&t, &p);
    Serial.print(t);
    Serial.print(" C  ");
    Serial.print(p);
    Serial.println(" Pa");
    delay(500);
}
