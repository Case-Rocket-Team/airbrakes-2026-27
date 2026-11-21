#pragma once

#include "ubx_messages.h"

bool gps_init();
void gps_update();
bool gps_has_fix();
const NavPvt* gps_latest();
uint32_t gps_fix_age_ms();

// Offset of `now` from `origin`, in feet (up = higher than origin).
struct GpsDelta {
    float north_ft;
    float east_ft;
    float up_ft;
};

GpsDelta gps_delta(NavPvt origin, NavPvt now);