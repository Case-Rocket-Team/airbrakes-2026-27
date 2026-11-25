#include "gps.h"
#include <Arduino.h>
void setup() {
    Serial.begin(115200);
    Serial.println(gps_init() ? "GPS OK" : "GPS not responding");
}

void loop() {
    gps_update();

    // const NavPvt* p = gps_latest();
    // Serial.println(p->num_sv);
    static uint32_t last_print = 0;
    if (millis() - last_print >= 200) {
        last_print = millis();
        if (gps_has_fix()) {
            const NavPvt* p = gps_latest();
            Serial.print(p->h_msl_mm / 1000.0f );
            Serial.print("\n");
        }
    }
}