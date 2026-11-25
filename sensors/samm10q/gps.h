#pragma once

#include "ubx_messages.h"

bool gps_init();
void gps_update();
bool gps_has_fix();
const NavPvt* gps_latest();
uint32_t gps_fix_age_ms();

// Offset of `now` from `origin`, in meters (up = higher than origin).
struct GpsDelta {
    float north_m;
    float east_m;
    float up_m;
};

GpsDelta gps_delta(NavPvt origin, NavPvt now);