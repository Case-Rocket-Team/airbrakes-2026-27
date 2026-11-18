#include "ubx_messages.h"
#include <cstdint>
#include <cstring>

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

