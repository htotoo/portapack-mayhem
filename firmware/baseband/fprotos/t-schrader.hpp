#ifndef __FPROTO_TPMS_SCHRADER_EG53MA4_H__
#define __FPROTO_TPMS_SCHRADER_EG53MA4_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSSchraderEG53MA4 : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSSchraderEG53MA4() {
        sensorType = FPT_Schrader;
        te_short = 120;
        te_long = 240;
        te_delta = 60;
    }

    bool sanity_check_eg53ma4(uint8_t* b) {
        uint8_t sum = 0;
        for (int i = 0; i < 9; i++) {
            sum += b[i];
        }
        if (sum != b[9]) return false;
        if (!b[1] && !b[2] && !b[4] && !b[5] && !b[7] && !b[8]) return false;
        if (b[4] == 0x00 && b[5] == 0x00 && b[6] == 0x00) return false;
        if (b[4] == 0xFF && b[5] == 0xFF && b[6] == 0xFF) return false;
        return true;
    }

    void analyze_eg53ma4(uint8_t* b) {
        id = (b[4] << 16) | (b[5] << 8) | b[6];
        pressure = (float)b[7] * 2.5f;                        // kpa
        temperature = ((float)b[8] - 32.0f) * (5.0f / 9.0f);  // celsius
        battery = 0xFF;                                       // no battery info in this proto
    }

    void feed(bool level, uint32_t duration) {
        if (level == false && duration > 400) {
            if (decode_count_bit >= 100) {
                bool found = false;
                for (int offset = 0; offset <= 16 && !found; offset++) {
                    uint8_t b[10];
                    uint8_t b_inv[10];
                    uint64_t d1 = decode_data >> offset;
                    if (offset > 0) {
                        uint64_t mask = (1ULL << offset) - 1;
                        d1 |= (decode_data2 & mask) << (64 - offset);
                    }
                    uint64_t d2 = decode_data2 >> offset;

                    // Payload (80 bit)
                    b[0] = (d2 >> 8) & 0xFF;
                    b[1] = (d2) & 0xFF;
                    for (int i = 0; i < 8; i++) {
                        b[i + 2] = (d1 >> (56 - i * 8)) & 0xFF;
                    }

                    for (int i = 0; i < 10; i++) b_inv[i] = ~b[i];
                    uint8_t preamble_raw = (d2 >> 16) & 0xFF;
                    if (preamble_raw == 0xFF && sanity_check_eg53ma4(b_inv)) {
                        data_count_bit = 80;
                        decode_data2 = (b_inv[0] << 8) | b_inv[1];
                        decode_data = 0;
                        for (int i = 0; i < 8; i++) {
                            decode_data = (decode_data << 8) | b_inv[i + 2];
                        }

                        analyze_eg53ma4(b_inv);
                        if (callback) callback(this);
                        found = true;
                    } else if (preamble_raw == 0x00 && sanity_check_eg53ma4(b)) {
                        data_count_bit = 80;
                        decode_data2 = (b[0] << 8) | b[1];
                        decode_data = 0;
                        for (int i = 0; i < 8; i++) {
                            decode_data = (decode_data << 8) | b[i + 2];
                        }
                        analyze_eg53ma4(b);
                        if (callback) callback(this);
                        found = true;
                    }
                }
            }

            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        ManchesterEvent event = ManchesterEventReset;
        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            if (decode_count_bit > 0) {
                FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                decode_count_bit = 0;
                decode_data = 0;
                decode_data2 = 0;
            }
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