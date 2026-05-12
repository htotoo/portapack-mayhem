#ifndef __FPROTO_TPMS_BMW_H__
#define __FPROTO_TPMS_BMW_H__

#include "subtpmsbase.hpp"

#define MODEL_AUDI 1
#define MODEL_BMW 2

class FProtoSubTPMSBMW : public FProtoSubTPMSBase {
   public:
    uint32_t decode_data2 = 0;

    FProtoSubTPMSBMW() {
        sensorType = FPT_BMW;
        te_short = 25;
        te_long = 50;
        te_delta = 12;
        min_count_bit_for_found = 64;
    }

    void tpms_protocol_bmw_analyze(uint8_t* b, int type) {
        if (type == MODEL_AUDI) {
            id = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];
            pressure = (float)b[5] * 0.0245f;
            temperature = (float)b[6] - 52.0f;
            sensorType = FPT_AUDI;
        } else {
            id = ((uint32_t)b[4] << 24) | ((uint32_t)b[5] << 16) | ((uint32_t)b[6] << 8) | b[7];
            pressure = (float)b[8] * 0.0245f;
            temperature = (float)b[9] - 52.0f;
            sensorType = FPT_BMW;
        }
        battery = 0xFF;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // END OF MESSAGE KIÉRTÉKELÉS
            if (decode_count_bit >= 64) {
                bool found = false;
                for (int offset = 0; offset <= 4 && !found; offset++) {
                    // 1. Audi Check (64 bit)
                    uint64_t d = decode_data >> offset;
                    if (d != 0ULL && d != ~0ULL) {
                        uint8_t b[8], b_inv[8];
                        for (int i = 0; i < 8; i++) {
                            b[i] = (d >> ((7 - i) * 8)) & 0xFF;
                            b_inv[i] = ~b[i];
                        }
                        if (FProtoGeneral::subghz_protocol_blocks_crc8(b, 7, 0x2F, 0xAA) == b[7]) {
                            data_count_bit = 64;
                            tpms_protocol_bmw_analyze(b, MODEL_AUDI);
                            if (callback) callback(this);
                            found = true;
                        } else if (FProtoGeneral::subghz_protocol_blocks_crc8(b_inv, 7, 0x2F, 0xAA) == b_inv[7]) {
                            data_count_bit = 64;
                            tpms_protocol_bmw_analyze(b_inv, MODEL_AUDI);
                            if (callback) callback(this);
                            found = true;
                        }
                    }

                    // 2. BMW Check (88 bit) - Csak ha nem talált Audit
                    if (!found && decode_count_bit >= 88) {
                        uint8_t b[11], b_inv[11];
                        uint64_t d_hi = decode_data2;
                        uint64_t d_lo = decode_data;
                        if (offset > 0) {
                            d_lo = (decode_data >> offset) | (decode_data2 << (64 - offset));
                            d_hi = decode_data2 >> offset;
                        }

                        b[0] = (d_hi >> 16) & 0xFF;
                        b[1] = (d_hi >> 8) & 0xFF;
                        b[2] = d_hi & 0xFF;
                        for (int i = 0; i < 8; i++) b[i + 3] = (d_lo >> ((7 - i) * 8)) & 0xFF;
                        for (int i = 0; i < 11; i++) b_inv[i] = ~b[i];

                        if (FProtoGeneral::subghz_protocol_blocks_crc8(b, 10, 0x2F, 0xAA) == b[10]) {
                            data_count_bit = 88;
                            tpms_protocol_bmw_analyze(b, MODEL_BMW);
                            if (callback) callback(this);
                            found = true;
                        } else if (FProtoGeneral::subghz_protocol_blocks_crc8(b_inv, 10, 0x2F, 0xAA) == b_inv[10]) {
                            data_count_bit = 88;
                            tpms_protocol_bmw_analyze(b_inv, MODEL_BMW);
                            if (callback) callback(this);
                            found = true;
                        }
                    }
                }
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
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif