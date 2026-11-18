#include "ubx_frame.h"

#include <cstdint>
#include <cstddef>
#include <cstring>

/*
    2 Preamble Bytes
    1 Message Class Byte
    1 Message ID Byte
    2 Length Bytes (Little Endian)
    N Payload Bytes
    2 Checksum Bytes (ckA, ckB)
*/
size_t ubxEncode(const UbxFrame& f, uint8_t* out, size_t outSize) {
    if (f.len > UBX_MAX_PAYLOAD || f.len + 8 > outSize) return 0; // Invalid 


    // Magic preamble bytes
    out[0] = 0xb5;          
    out[1] = 0x62;     

    out[2] = f.cls;         
    out[3] = f.id;      
    
    // UBX protocol defines every multi-byte field as little-endian

    out[4] = f.len & 0xFF; // Low byte
    out[5] = (f.len >> 8);  // High byte

    memcpy(out + 6, f.payload, f.len); // Copy payload

    // Checksum bytes ckA and ckB
    uint8_t ckA, ckB;

    ubxChecksum(out + 2, 4 + f.len, ckA, ckB);

    out[f.len + 6] = ckA;
    out[f.len + 7] = ckB;

    return 8 + f.len;

}

// Fletcher Checksum Algorithm
void ubxChecksum(const uint8_t* buffer, const size_t len, uint8_t& ckA, uint8_t& ckB) {
    ckA = 0;
    ckB = 0;

    for (size_t i = 0; i < len; i++) {
        ckA = ckA + buffer[i];
        ckB += ckA;
    }

}
