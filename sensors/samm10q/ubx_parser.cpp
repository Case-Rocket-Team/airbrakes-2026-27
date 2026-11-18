#include "ubx_parser.h"

static void ck_update(UbxParser* p, uint8_t b) {
    p->ckA += b;
    p->ckB += p->ckA;
}

void ubx_parser_init(UbxParser* p) {
    p->state = UBX_STATE_SYNC1;
    p->idx = 0;
    p->ckA = 0;
    p->ckB = 0;
}

bool ubx_parser_feed(UbxParser* p, uint8_t b) {
    switch (p->state) {
        case UBX_STATE_SYNC1:
            if (b == 0xB5) {
                p->state = UBX_STATE_SYNC2;
            }
            break;

        case UBX_STATE_SYNC2:
            if (b == 0x62) {
                p->state = UBX_STATE_CLASS;
            } else if (b == 0xB5) { // could be the real start, so stay in SYNC2
                p->state = UBX_STATE_SYNC2;
            } else {
                p->state = UBX_STATE_SYNC1;
            }
            break;

        case UBX_STATE_CLASS: // checksum accumulation starts at class byte
            p->ckA = 0;
            p->ckB = 0;
            ck_update(p, b);
            p->frame.cls = b;
            p->state = UBX_STATE_ID;
            break;

        case UBX_STATE_ID:
            ck_update(p, b);
            p->frame.id = b;
            p->state = UBX_STATE_LEN1;
            break;

        case UBX_STATE_LEN1:
            ck_update(p, b);
            p->frame.len = b;
            p->state = UBX_STATE_LEN2;
            break;

        case UBX_STATE_LEN2:
            ck_update(p, b);
            p->frame.len |= (b << 8);
            p->idx = 0;

            if (p->frame.len > UBX_MAX_PAYLOAD) { // invalid payload
                p->state = UBX_STATE_SYNC1;
                break;
            }

            if (p->frame.len == 0) { // no payload, skip to checksum
                p->state = UBX_STATE_CKA;
                break;
            }

            p->state = UBX_STATE_PAYLOAD;
            break;

        case UBX_STATE_PAYLOAD:
            ck_update(p, b);
            p->frame.payload[p->idx] = b;
            p->idx++;
            if (p->idx == p->frame.len) {
                p->state = UBX_STATE_CKA;
            }
            break;

        case UBX_STATE_CKA: // if checksum matches, go to CKB, otherwise SYNC1
            p->state = (b == p->ckA) ? UBX_STATE_CKB : UBX_STATE_SYNC1;
            break;

        case UBX_STATE_CKB: // frame complete if checksum matches
            p->state = UBX_STATE_SYNC1;
            return b == p->ckB;
    }
    return false;
}