#include "ubx_messages.h"
#include "ubx_frame.h"
#include <cstdint>

#define UBX_NAV_PVT_LEN 92

// Little-endian helpers
static uint16_t rd_u16(const uint8_t* p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t rd_u32(const uint8_t* p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t* p) {
    return (int32_t)rd_u32(p);
}

static void wr_u16(uint8_t* p, uint16_t v) {
    p[0] =  v       & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

static void wr_u32(uint8_t* p, uint32_t v) {
    p[0] =  v        & 0xFF;
    p[1] = (v >> 8)  & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
}

void ubx_build_poll(UbxFrame* f, uint8_t cls, uint8_t id) {
    f->cls = cls;
    f->id = id;
    f->len = 0; // Poll frame, so no payload
}

// 3.9
int ubx_check_ack(const UbxFrame* f, uint8_t cls, uint8_t id) {
    if (f->cls != UBX_CLASS_ACK || f->len != 2) { // not an ACK response, or malformed
        return 0;
    }

    if (f->payload[0] != cls || f->payload[1] != id) { // ACK, but not for this specific request
        return 0;
    }

    // Response is part of the ACK class, and is meant for this specific request

    if (f->id == UBX_ID_ACK_ACK) return 1;
    if (f->id == UBX_ID_ACK_NAK) return -1;
    return 0;   // class 0x05 but an unknown id

}

/*
 * CFG-VALSET (3.10.5.1) sets configuration values on the receiver.
 * Payload: version (0), layers (which memory to write, bit 0 = RAM),
 * 2 reserved bytes, then key/value pairs packed back to back. Each key is
 * 4 bytes, little-endian, and encodes the size of the value that follows it.
 * The module replies ACK if all values were applied, or NAK and applies none.
 */

void ubx_valset_begin(UbxFrame* f, uint8_t layers) {
    f->cls = UBX_CLASS_CFG;
    f->id = UBX_ID_CFG_VALSET;
    f->payload[0] = 0x00; // message version 0 (no transactions)
    f->payload[1] = layers;

    // Two reserved bytes
    f->payload[2] = 0x00;
    f->payload[3] = 0x00; 

    f->len = 4; // header is the only payload currently
}

// appends a key with a 1 byte value
bool ubx_valset_add_u8(UbxFrame* f, uint32_t key, uint8_t value) {

    if (f->len + 4 + 1 > UBX_MAX_PAYLOAD) {
        return false; // Exceeded max frame length
    }

    wr_u32(&f->payload[f->len], key); // 4 byte key, little-endian
    f->payload[f->len + 4] = value;

    f->len += 4 + 1; // 4 byte key, 1 byte value (u8)

    return true;
}
// Appends a key with a 2 byte value
bool ubx_valset_add_u16(UbxFrame* f, uint32_t key, uint16_t value) {
    if (f->len + 4 + 2 > UBX_MAX_PAYLOAD) {
        return false; // Exceeded max frame length
    }

    wr_u32(&f->payload[f->len], key);
    wr_u16(&f->payload[f->len + 4], value);

    f->len += 4 + 2; // 4 byte key, 2 byte value (u16)

    return true;
}

// Appends a key with a 4 byte value
bool ubx_valset_add_u32(UbxFrame* f, uint32_t key, uint32_t value) {
    if (f->len + 4 + 4 > UBX_MAX_PAYLOAD) {
        return false; // Exceeded max frame length
    }

    wr_u32(&f->payload[f->len], key);
    wr_u32(&f->payload[f->len + 4], value);

    f->len += 4 + 4; // 4 byte key, 4 byte value (u32)
    
    return true;
}


// I HIGHLY advise refering to 3.15.11.1 if you want this to make any sense.
NavPvt ubx_decode_nav_pvt(const UbxFrame* f) {
    NavPvt out = {};

    if (f->cls != UBX_CLASS_NAV || f->id != UBX_ID_NAV_PVT) { // not a nav_pvt response
        return out;
    }

    if (f->len != UBX_NAV_PVT_LEN) { // malformed request
        return out;
    }

    out.i_tow_ms = rd_u32(&f->payload[0]);
    out.fix_type = f->payload[20];
    out.gnss_fix_ok = (f->payload[21] & 0x01) != 0;
    out.num_sv = f->payload[23];

    out.lon_deg_e7 = rd_i32(&f->payload[24]);
    out.lat_deg_e7 = rd_i32(&f->payload[28]);
    out.h_msl_mm = rd_i32(&f->payload[36]);


    out.vel_n_mm_s = rd_i32(&f->payload[48]);
    out.vel_e_mm_s = rd_i32(&f->payload[52]);
    out.vel_d_mm_s = rd_i32(&f->payload[56]);

    out.h_acc_mm = rd_u32(&f->payload[40]);
    out.v_acc_mm = rd_u32(&f->payload[44]);

    out.valid = true;
    return out;
}