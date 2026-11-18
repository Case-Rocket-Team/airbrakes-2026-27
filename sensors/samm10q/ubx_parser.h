#pragma once

#include "ubx_frame.h"
#include <cstddef>
#include <cstdint>

enum UbxState {
    UBX_STATE_SYNC1,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN1,
    UBX_STATE_LEN2,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CKA,
    UBX_STATE_CKB
};

struct UbxParser {
    UbxState state = UBX_STATE_SYNC1; // Current frame location
    UbxFrame frame; // Current frame
    size_t   idx = 0; // # payload bytes recieved
    uint8_t  ckA = 0; // Checksum A
    uint8_t  ckB = 0; // Checksum B
};

