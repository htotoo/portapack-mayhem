#ifndef __FPROTO_TPMS_BMW_H__
#define __FPROTO_TPMS_BMW_H__

#include "subtpmsbase.hpp"

#define MODEL_AUDI 1
#define MODEL_BMW 2

class FProtoSubTPMSBMW : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSBMW() {
        sensorType = FPT_BMW;
        te_short = 25;
        te_long = 50;
        te_delta = 12;
        min_count_bit_for_found = 64;
    }

    void tpms_protocol_bmw_analyze() {
        uint8_t b[11] = {0};

        if (saved_type == MODEL_AUDI) {
            for (int i = 0; i < 8; i++) b[i] = (saved_data >> ((7 - i) * 8)) & 0xFF;
            sensorType = FPT_AUDI;
        } else if (saved_type == MODEL_BMW) {
            b[0] = (saved_data2 >> 16) & 0xFF;
            b[1] = (saved_data2 >> 8) & 0xFF;
            b[2] = (saved_data2) & 0xFF;
            for (int i = 0; i < 8; i++) b[3 + i] = (saved_data >> ((7 - i) * 8)) & 0xFF;
            sensorType = FPT_BMW;
        }

        id = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];
        pressure = (float)b[5] * 0.0245f;
        temperature = (float)b[6] - 52.0f;
        battery = 0xFF;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            if (packet_ready) {
                data_count_bit = (saved_type == MODEL_BMW) ? 88 : 64;
                tpms_protocol_bmw_analyze();
                if (callback) callback(this);
                packet_ready = false;
            }
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        bool bitstate;
        if (FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate)) {
            decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;

            // ==========================================
            // BMW FAST EXIT
            // ==========================================
            if (decode_data == 0ULL || decode_data == ~0ULL) return;

            if (decode_count_bit >= 64) {
                uint8_t b[8], b_inv[8];
                for (int i = 0; i < 8; i++) {
                    b[i] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                    b_inv[i] = ~b[i];
                }
                if (FProtoGeneral::subghz_protocol_blocks_crc8(b, 7, 0x2F, 0xAA) == b[7]) {
                    saved_data = decode_data;
                    saved_type = MODEL_AUDI;
                    packet_ready = true;
                } else if (FProtoGeneral::subghz_protocol_blocks_crc8(b_inv, 7, 0x2F, 0xAA) == b_inv[7]) {
                    saved_data = ~decode_data;
                    saved_type = MODEL_AUDI;
                    packet_ready = true;
                }
            }

            if (decode_count_bit >= 88) {
                uint8_t b[11], b_inv[11];
                b[0] = (decode_data2 >> 16) & 0xFF;
                b[1] = (decode_data2 >> 8) & 0xFF;
                b[2] = (decode_data2) & 0xFF;
                b_inv[0] = ~b[0];
                b_inv[1] = ~b[1];
                b_inv[2] = ~b[2];
                for (int i = 0; i < 8; i++) {
                    b[i + 3] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                    b_inv[i + 3] = ~b[i + 3];
                }

                if (FProtoGeneral::subghz_protocol_blocks_crc8(b, 10, 0x2F, 0xAA) == b[10]) {
                    saved_data = decode_data;
                    saved_data2 = decode_data2;
                    saved_type = MODEL_BMW;
                    packet_ready = true;
                } else if (FProtoGeneral::subghz_protocol_blocks_crc8(b_inv, 10, 0x2F, 0xAA) == b_inv[10]) {
                    saved_data = ~decode_data;
                    saved_data2 = ~decode_data2;
                    saved_type = MODEL_BMW;
                    packet_ready = true;
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
    uint64_t saved_data = 0;
    uint64_t saved_data2 = 0;
    int saved_type = 0;
    bool packet_ready = false;
};

#endif