#ifndef __FPROTO_TPMS_FORD_H__
#define __FPROTO_TPMS_FORD_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSFord : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSFord() {
        sensorType = FPT_Ford;
        te_short = 52;
        te_long = 104;
        te_delta = 25;
        min_count_bit_for_found = 64;
    }

    void tpms_protocol_ford_analyze(uint8_t* b) {
        id = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
        battery = 0xFF;
        int psibits = (((b[6] & 0x20) << 3) | b[4]);
        pressure = (psibits * 0.25f) * 0.068947f;
        if ((b[5] & 0x80) == 0) temperature = (b[5] & 0x7f) - 56;
    }

    bool ford_payload_sanity_check(uint8_t* b) {
        uint8_t sum = 0;
        for (int i = 0; i < 7; i++) sum += b[i];
        if (sum != b[7]) return false;

        uint8_t flags = b[6];
        if ((flags & 0x90) != 0x00) return false;
        uint8_t state = flags & 0x4C;
        if (state != 0x08 && state != 0x04 && state != 0x44) return false;
        if (b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0) return false;

        return true;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // ========================================================
            // END OF MESSAGE (SQUELCH ZÁR, VAGY HIBÁS IMPULZUS)
            // Csak itt fut le a nehéz matematika, pontosan 1 alkalommal!
            // ========================================================
            if (decode_count_bit >= 64) {
                bool found = false;
                // Visszanézünk pár bitet, ha zaj volt a csomag legvégén
                for (int offset = 0; offset <= 4 && !found; offset++) {
                    uint8_t b[8], b_inv[8];
                    uint64_t d = decode_data >> offset;

                    for (int i = 0; i < 8; i++) {
                        b[i] = (d >> ((7 - i) * 8)) & 0xFF;
                        b_inv[i] = ~b[i];
                    }

                    if (ford_payload_sanity_check(b)) {
                        data_count_bit = 64;
                        tpms_protocol_ford_analyze(b);
                        if (callback) callback(this);
                        found = true;
                    } else if (ford_payload_sanity_check(b_inv)) {
                        data_count_bit = 64;
                        tpms_protocol_ford_analyze(b_inv);
                        if (callback) callback(this);
                        found = true;
                    }
                }
            }

            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            return;
        }

        // AMÍG A JEL TART, CSAK A BITEKET TOLJUK (Zéró CPU teher)
        bool bitstate;
        if (FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate)) {
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif