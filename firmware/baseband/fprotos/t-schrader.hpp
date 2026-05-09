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
        min_count_bit_for_found = 115;
    }

    bool sanity_check_eg53ma4(uint8_t* b) {
        // 1. Modulo-256 Checksum
        uint8_t sum = 0;
        for (int i = 0; i < 9; i++) {
            sum += b[i];
        }
        if (sum != b[9]) return false;

        // 2. Prevent dead-air / zeroed matches
        if (!b[1] && !b[2] && !b[4] && !b[5] && !b[7] && !b[8]) return false;
        if (b[4] == 0x00 && b[5] == 0x00 && b[6] == 0x00) return false;
        if (b[4] == 0xFF && b[5] == 0xFF && b[6] == 0xFF) return false;

        // 3. Strict physical limits
        if (b[7] > 240) return false;
        if (b[8] > 212) return false;

        return true;
    }

    void analyze_eg53ma4(uint8_t* b) {
        id = (b[4] << 16) | (b[5] << 8) | b[6];
        pressure = (float)b[7] * 2.5f;
        temperature = ((float)b[8] - 32.0f) * (5.0f / 9.0f);
        battery = 0xFF;
    }

    void feed(bool level, uint32_t duration) {
        // 1. GAP DETECTOR
        if (level == false && duration > 450) {
            if (decode_count_bit >= 115) {
                bool found = false;

                // CRITICAL FIX: Scan backwards (3 down to 0) to evaluate the true, older packet
                // BEFORE the left-shifted "ghost" caused by trailing RF noise.
                for (int offset = 3; offset >= 0 && !found; offset--) {
                    uint8_t b[10];
                    uint8_t b_inv[10];

                    uint64_t d1 = decode_data >> offset;

                    // Prevent C++ Undefined Behavior when offset is 0
                    if (offset > 0) {
                        uint64_t mask = (1ULL << offset) - 1;
                        d1 |= (decode_data2 & mask) << (64 - offset);
                    }
                    uint64_t d2 = decode_data2 >> offset;

                    b[0] = (d2 >> 8) & 0xFF;
                    b[1] = (d2) & 0xFF;
                    for (int i = 0; i < 8; i++) {
                        b[i + 2] = (d1 >> (56 - i * 8)) & 0xFF;
                    }

                    for (int i = 0; i < 10; i++) b_inv[i] = ~b[i];

                    if (sanity_check_eg53ma4(b)) {
                        data_count_bit = 80;
                        analyze_eg53ma4(b);
                        if (callback) callback(this);
                        found = true;
                    } else if (sanity_check_eg53ma4(b_inv)) {
                        data_count_bit = 80;
                        analyze_eg53ma4(b_inv);
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

        // 2. TIMING CLASSIFIER
        ManchesterEvent event = ManchesterEventReset;
        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            return;
        }

        // 3. MANCHESTER DECODER
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