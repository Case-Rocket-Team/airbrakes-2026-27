#pragma once

#include <cstdint>
#include <cstddef>

constexpr size_t UBX_MAX_PAYLOAD = 256;

struct UbxFrame {
    uint8_t cls = 0x00;
    uint8_t id = 0x00;
    uint16_t len = 0x00;
    uint8_t payload[UBX_MAX_PAYLOAD]{};
};

size_t ubx_encode(const UbxFrame* f, uint8_t* out, size_t outSize);
void ubx_checksum(const uint8_t* buffer, size_t len, uint8_t& ck_a, uint8_t& ck_b);