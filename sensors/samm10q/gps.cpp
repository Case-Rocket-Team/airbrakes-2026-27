#include "gps.h"
#include "ubx_parser.h"
#include <Arduino.h>

#define GPS_SERIAL Serial1
#define GPS_BAUD 115200 
#define GPS_SERIAL_BUFFER_PAGE_LENGTH 512 
#define GPS_ACK_TIMEOUT_MS 500
#define GPS_FIX_TIMEOUT_MS 1000

#define EARTH_RADIUS_M 6371000.0
#define DEG_E7_TO_RAD (1e-7 * PI / 180.0)
#define MM_TO_M 0.001

static UbxParser parser = {};
static NavPvt latest = {};
static bool has_fix = false;            // most recent NAV-PVT had a valid 3D fix

static uint32_t last_fix_time_ms = 0;   // last NAV-PVT with a valid fix (0 = never)
static uint32_t last_pvt_time_ms = 0;   // last NAV-PVT of any kind

// Encodes a UBX frame and writes it to the GPS serial port.
// Returns false if the frame couldn't be encoded (payload too large).
static bool send_frame(const UbxFrame* f) {
    uint8_t buf[UBX_MAX_PAYLOAD + 8];
    size_t n = ubx_encode(f, buf, sizeof buf); 
    if (n == 0) {
        return false; // invalid frame, nothing sent
    }
    GPS_SERIAL.write(buf, n); // send exactly n bytes
    return true;
}

// Waits for the module's ACK or NAK for a previously sent message (cls/id).
// Ignores unrelated frames. Blocks until an answer arrives or timeout_ms passes.
// 1 = ACK, -1 = NAK, 0 = timeout.
static int wait_for_ack(uint8_t cls, uint8_t id, uint32_t timeout_ms) {
    uint32_t start = millis();
    
    while (millis() - start < timeout_ms) {
        while (GPS_SERIAL.available()) {
            if (ubx_parser_feed(&parser, GPS_SERIAL.read())) {
                int r = ubx_check_ack(&parser.frame, cls, id);
                if (r != 0) {
                    return r;
                }
            }
        }
    }
    return 0;
}	

static bool send_and_ack(const UbxFrame* f)	{
    if (!send_frame(f)) { // send frame
        return false;
    }
    return wait_for_ack(f->cls, f->id, GPS_ACK_TIMEOUT_MS) == 1; // Wait for ack
}

// Polls the module for its firmware version (UBX-MON-VER) and waits for the
// reply. 
static bool poll_mon_ver(uint32_t timeout_ms) {
    UbxFrame tx;
    ubx_build_poll(&tx, UBX_CLASS_MON, UBX_ID_MON_VER);
    if (!send_frame(&tx)) { // couldn't send
        return false;
    }

    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        while (GPS_SERIAL.available()) {
            if (ubx_parser_feed(&parser, GPS_SERIAL.read())) {
                // valid frame! :D
                const UbxFrame* f = &parser.frame;
                if (f->cls == UBX_CLASS_MON && f->id == UBX_ID_MON_VER) {
                    // we're in business
                    return true;
                }
            }
        }
    }

    return false;

}

static void handle_frame(const UbxFrame* f) {
    NavPvt pvt = ubx_decode_nav_pvt(f);
    if (!pvt.valid) {
        return; // not a NAV-PVT frame
    }

    pvt.frame_read_us = micros(); // frame's last byte was just parsed

    last_pvt_time_ms = millis(); // GPS is still talking

    has_fix = pvt.gnss_fix_ok && pvt.fix_type == 3; // only trust 3D fixes

    if (has_fix) {
        latest = pvt;
        last_fix_time_ms = millis();
    }
}

bool gps_init() {
    ubx_parser_init(&parser);
    GPS_SERIAL.begin(9600); // SAM-M10Q default baud

    // Default RX buffer is 64 bytes, smaller than one 100-byte NAV-PVT frame.
    static uint8_t rx_extra[GPS_SERIAL_BUFFER_PAGE_LENGTH];
    GPS_SERIAL.addMemoryForRead(rx_extra, sizeof rx_extra);
    delay(500); // let the module boot

    if (!poll_mon_ver(1000)) {
        return false; // module not responding
    }

    // Switch the module to GPS_BAUD, then follow it.
    UbxFrame baud;
    ubx_valset_begin(&baud, UBX_LAYER_RAM);
    ubx_valset_add_u32(&baud, CFG_UART1_BAUDRATE, GPS_BAUD);
    send_frame(&baud);
    GPS_SERIAL.flush(); // wait until the last byte has been sent
    delay(100); // give the module time to switch
    GPS_SERIAL.begin(GPS_BAUD); // clear the RX buffer

    UbxFrame tx;
    ubx_valset_begin(&tx, UBX_LAYER_RAM);
    ubx_valset_add_u8(&tx, CFG_UART1OUTPROT_UBX, 1); // make sure UBX is on
    ubx_valset_add_u8(&tx, CFG_MSGOUT_UBX_NAV_PVT_UART1, 1); // NAV-PVT every solution
    ubx_valset_add_u8(&tx, CFG_UART1OUTPROT_NMEA, 0); // NMEA off
    ubx_valset_add_u16(&tx, CFG_RATE_MEAS, 200); // 5 Hz
    ubx_valset_add_u8(&tx, CFG_NAVSPG_DYNMODEL, DYNMODEL_AIR4);

    return send_and_ack(&tx);
}

void gps_update() {
    while (GPS_SERIAL.available()) {
        if (ubx_parser_feed(&parser, GPS_SERIAL.read())) {
            handle_frame(&parser.frame);
        }
    }
}
// True if the latest solution is a 3D fix.
bool gps_has_fix() {
    return has_fix && gps_fix_age_ms() < GPS_FIX_TIMEOUT_MS;
}

const NavPvt* gps_latest() {
	return &latest;
}

uint32_t gps_fix_age_ms() {
    if (last_fix_time_ms == 0) {
        return UINT32_MAX; // no fix received yet: treat as infinitely old
    }

    return millis() - last_fix_time_ms;
}


GpsDelta gps_delta(NavPvt origin, NavPvt now) {
    int32_t d_lat_e7 = now.lat_deg_e7 - origin.lat_deg_e7;
    int32_t d_lon_e7 = now.lon_deg_e7 - origin.lon_deg_e7;
    int32_t d_h_mm = now.h_msl_mm - origin.h_msl_mm;

    // A degree of longitude gets shorter away from the equator
    double cos_lat = cos(origin.lat_deg_e7 * DEG_E7_TO_RAD);

    GpsDelta d = {
        (float)(d_lat_e7 * DEG_E7_TO_RAD * EARTH_RADIUS_M),
        (float)(d_lon_e7 * DEG_E7_TO_RAD * EARTH_RADIUS_M * cos_lat),
        (float)(d_h_mm * MM_TO_M),
    };
    return d;
}
