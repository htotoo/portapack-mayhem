#ifndef __FPROTO_TPMS_ELANTRA2012_H__
#define __FPROTO_TPMS_ELANTRA2012_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSElantra2012 : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSElantra2012() {
        sensorType = FPT_Elantra2012;
        te_short = 52;
        te_long = 104;
        te_delta = 35;
        min_count_bit_for_found = 64;
    }

    bool checksum_elantra(uint8_t* b) {
        uint8_t crc = 0x00;
        for (int i = 0; i < 8; i++) {
            crc ^= b[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x07;
                else
                    crc <<= 1;
            }
        }
        return (crc == 0x00);
    }

    void analyze_elantra(uint8_t* b) {
        pressure = (float)(b[0] + 60);
        temperature = (float)(b[1] - 50);
        id = ((uint32_t)b[2] << 24) | ((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5];
        int battery_low = (b[6] & 0x02) >> 1;
        battery = battery_low ? 10 : 100;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        uint32_t mid = (te_short + te_long) / 2;

        if (duration >= (te_short - te_delta) && duration <= mid) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (duration > mid && duration <= (te_long + te_delta)) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            return;
        }

        bool bitstate;
        if (FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate)) {
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;

            if (decode_count_bit >= min_count_bit_for_found) {
                if (decode_data == 0ULL || decode_data == ~(0ULL)) return;
                uint8_t b[8];
                uint8_t b_inv[8];

                for (int i = 0; i < 8; i++) {
                    b[i] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                    b_inv[i] = ~b[i];
                }

                if (checksum_elantra(b)) {
                    data_count_bit = 64;
                    analyze_elantra(b);
                    if (callback) callback(this);

                    decode_count_bit = 0;
                    decode_data = 0;
                    FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                } else if (checksum_elantra(b_inv)) {
                    data_count_bit = 64;
                    analyze_elantra(b_inv);
                    if (callback) callback(this);

                    decode_count_bit = 0;
                    decode_data = 0;
                    FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif