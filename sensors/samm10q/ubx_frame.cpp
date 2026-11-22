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
    2 Checksum Bytes (ck_a, ck_b)
*/
size_t ubx_encode(const UbxFrame* f, uint8_t* out, size_t outSize) {
    if (f->len > UBX_MAX_PAYLOAD || f->len + 8 > outSize) return 0; // Invalid 

    // Magic preamble bytes
    out[0] = 0xb5;          
    out[1] = 0x62;     

    out[2] = f->cls;         
    out[3] = f->id;      
    
    // UBX protocol defines every multi-byte field as little-endian

    out[4] = f->len & 0xFF; // Low byte
    out[5] = (f->len >> 8);  // High byte

    memcpy(out + 6, f->payload, f->len); // Copy payload

    // Checksum bytes ck_a and ck_b
    UbxChecksum ck = ubx_checksum(out + 2, 4 + f->len);

    out[f->len + 6] = ck.ck_a;
    out[f->len + 7] = ck.ck_b;

    return 8 + f->len;

}

// Fletcher Checksum Algorithm
UbxChecksum ubx_checksum(const uint8_t* buffer, const size_t len) {
    UbxChecksum ck = { 0, 0 };

    for (size_t i = 0; i < len; i++) {
        ck.ck_a += buffer[i];
        ck.ck_b += ck.ck_a;
    }

    return ck;
}
