#pragma once

#include "ubx_messages.h"

bool gps_init();
void gps_update();
bool gps_has_fix();
const NavPvt* gps_latest();
uint32_t gps_fix_age_ms();