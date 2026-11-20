#include "gps.h"
#include "ubx_parser.h"
#include <Arduino.h>
#include <SPI.h>


#define GPS_SERIAL Serial1
#define GPS_ACK_TIMEOUT_MS 500
#define GPS_FIX_TIMEOUT_MS 1000

static UbxParser parser;
static NavPvt latest;
static bool has_fix = false;

static uint32_t last_fix_time_ms = 0;   // last NAV-PVT with a valid fix
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
    if (!send_frame(&tx)) { // couldnt send
        return false;
    }

    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        while (GPS_SERIAL.available()) {
            if (ubx_parser_feed(&parser, GPS_SERIAL.read())) {
                // valid frame!
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
    NavPvt pvt;
    if (!ubx_decode_nav_pvt(f, &pvt)) {
        return;                         // not a NAV-PVT frame
    }

    last_pvt_time_ms = millis();        // GPS is still talking

    if (pvt.gnss_fix_ok && pvt.fix_type == 3) {   // only keep trustworthy 3D fixes
        latest = pvt;
        has_fix = true;
        last_fix_time_ms = millis();
    }
}

bool gps_init() {
    ubx_parser_init(&parser);
    GPS_SERIAL.begin(9600); // might be 38400?
    delay(500); // let the module boot

    if (!poll_mon_ver(1000)) {
        return false; // module not responding
    }

    UbxFrame tx;
    ubx_valset_begin(&tx, UBX_LAYER_RAM);
    ubx_valset_add_u8 (&tx, CFG_UART1OUTPROT_UBX, 1); // make sure UBX is on
    ubx_valset_add_u8 (&tx, CFG_MSGOUT_UBX_NAV_PVT_UART1, 1); // NAV-PVT every solution
    ubx_valset_add_u8 (&tx, CFG_UART1OUTPROT_NMEA, 0); // NMEA off
    ubx_valset_add_u16(&tx, CFG_RATE_MEAS, 200); // 5 Hz
    ubx_valset_add_u8 (&tx, CFG_NAVSPG_DYNMODEL, DYNMODEL_AIR4);

    return send_and_ack(&tx);
}

void gps_update() {
    while (GPS_SERIAL.available()) {
        if (ubx_parser_feed(&parser, GPS_SERIAL.read())) {
            handle_frame(&parser.frame);
        }
    }
}
bool gps_has_fix() {
	return gps_fix_age_ms() < GPS_FIX_TIMEOUT_MS;
}

const NavPvt* gps_latest() {
	return &latest;
}

uint32_t gps_fix_age_ms() {
    if (!has_fix) {
        return UINT32_MAX; // no fix received yet: treat as infinitely old
    }

	return millis() - last_fix_time_ms;
}
