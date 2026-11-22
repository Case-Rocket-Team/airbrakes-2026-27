#pragma once

#include <cstdint>
#include "ubx_frame.h"

// Message classes
#define UBX_CLASS_NAV      0x01
#define UBX_CLASS_ACK      0x05
#define UBX_CLASS_CFG      0x06
#define UBX_CLASS_MON      0x0A

// Message IDs: NAV
#define UBX_ID_NAV_PVT     0x07

// Message IDs: ACK
#define UBX_ID_ACK_NAK     0x00
#define UBX_ID_ACK_ACK     0x01

// Message IDs: CFG
#define UBX_ID_CFG_VALSET  0x8A

// Message IDs: MON
#define UBX_ID_MON_VER     0x04

// CFG-VALSET layers (3.10.5.1)
#define UBX_LAYER_RAM      0x01

// Config keys (chapter 4)
#define CFG_MSGOUT_UBX_NAV_PVT_UART1  0x20910007  // U1: Output rate (1 = every solution)
#define CFG_UART1OUTPROT_UBX          0x10740001  // L: UBX output on UART1
#define CFG_UART1OUTPROT_NMEA         0x10740002  // L: NMEA output on UART1
#define CFG_RATE_MEAS                 0x30210001  // U2: Measurement period, ms
#define CFG_NAVSPG_DYNMODEL           0x20110021  // E1: Dynamic platform model
#define CFG_UART1_BAUDRATE            0x40520001  // U4: Baud rate

#define DYNMODEL_AIR4                 8           // airborne, <4g acceleration

// Builders
UbxFrame ubx_build_poll(UbxFrame* f, uint8_t cls, uint8_t id);

void ubx_valset_begin(UbxFrame* f, uint8_t layers);
bool ubx_valset_add_u8(UbxFrame* f, uint32_t key, uint8_t value);
bool ubx_valset_add_u16(UbxFrame* f, uint32_t key, uint16_t value);
bool ubx_valset_add_u32(UbxFrame* f, uint32_t key, uint32_t value);

int ubx_check_ack(const UbxFrame* f, uint8_t cls, uint8_t id);

// UBX-NAV-PVT (3.15.11.1)
struct NavPvt {
    bool     valid = false;         // false if the frame was not a well-formed NAV-PVT
    uint32_t frame_read_us = 0;     // micros() when the frame finished parsing (set by gps.cpp)
    uint32_t i_tow_ms = 0;          // GPS time of week
    uint8_t  fix_type = 0;          // 0 = no fix, 2 = 2D, 3 = 3D
    bool     gnss_fix_ok = false;   // bit 0 of flags: fix is valid
    uint8_t  num_sv = 0;            // satellites used in the solution

    int32_t  lon_deg_e7 = 0;        // longitude, degrees * 1e7
    int32_t  lat_deg_e7 = 0;        // latitude, degrees * 1e7
    int32_t  h_msl_mm = 0;          // height above mean sea level

    int32_t  vel_n_mm_s = 0;        // north velocity
    int32_t  vel_e_mm_s = 0;        // east velocity
    int32_t  vel_d_mm_s = 0;        // down velocity (positive = descending)

    uint32_t h_acc_mm = 0;          // horizontal accuracy estimate
    uint32_t v_acc_mm = 0;          // vertical accuracy estimate
};
// Returns a zeroed NavPvt with valid = false if `f` is not a well-formed NAV-PVT frame.
NavPvt ubx_decode_nav_pvt(const UbxFrame* f);