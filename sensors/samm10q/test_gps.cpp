#include "gps.h"
#include <Arduino.h>
void setup() {
    Serial.begin(115200);
    Serial.println(gps_init() ? "GPS OK" : "GPS not responding");
}

void loop() {
    gps_update();

    static uint32_t last_print = 0;
    if (millis() - last_print >= 500) {
        last_print = millis();
        if (gps_has_fix()) {
            const NavPvt* p = gps_latest();
            Serial.printf("lat %ld  lon %ld  alt %ld mm  sats %u\n",
                          p->lat_deg_e7, p->lon_deg_e7, p->h_msl_mm, p->num_sv);
            Serial.print(p->h_msl_mm / 1000.0f );
        }
    }
}