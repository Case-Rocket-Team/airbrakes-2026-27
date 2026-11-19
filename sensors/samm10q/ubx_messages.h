#pragma once

#define UBX_CLASS_CFG     0x06
#define UBX_ID_CFG_VALSET 0x8A
#define UBX_CLASS_ACK     0x05
#define UBX_ID_ACK_ACK    0x01
#define UBX_ID_ACK_NAK    0x00


void ubx_build_poll(UbxFrame* f, uint8_t cls, uint8_t id);

int ubx_check_ack(const UbxFrame* f, uint8_t cls, uint8_t id);

void ubx_valset_begin(UbxFrame* f, uint8_t layers);
bool ubx_valset_add_u8(UbxFrame* f, uint32_t key, uint8_t value);
bool ubx_valset_add_u16(UbxFrame* f, uint32_t key, uint16_t value);
bool ubx_valset_add_u32(UbxFrame* f, uint32_t key, uint32_t value);