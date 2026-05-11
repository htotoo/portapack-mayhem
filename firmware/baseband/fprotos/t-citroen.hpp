#ifndef __FPROTO_TPMS_CITROEN_H__
#define __FPROTO_TPMS_CITROEN_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSCitroen : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSCitroen() {
        sensorType = FPT_Citroen;
        te_short = 52;
        te_long = 104;
        te_delta = 35;
    }
    bool checksum_citroen(uint8_t* b) {
        uint8_t crc = 0;
        for (int i = 1; i <= 9; i++) {
            crc ^= b[i];
        }
        return (crc == 0);
    }

    void analyze_citroen(uint8_t* b) {
        id = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];
        pressure = (float)b[6] * 1.364f;
        temperature = (float)b[7] - 50.0f;
        battery = b[8];
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
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        bool bitstate;
        if (FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate)) {
            decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
            decode_data = (decode_data << 1) | bitstate;
            uint16_t pre12 = (decode_data2 >> 16) & 0x0FFF;

            if (pre12 == 0x0001 || pre12 == 0x0FFE) {
                uint8_t b[10];
                b[0] = (decode_data2 >> 8) & 0xFF;
                b[1] = (decode_data2) & 0xFF;
                for (int i = 0; i < 8; i++) b[i + 2] = (decode_data >> (56 - i * 8)) & 0xFF;
                if (pre12 == 0x0FFE) {
                    for (int i = 0; i < 10; i++) b[i] = ~b[i];
                }
                if (checksum_citroen(b)) {
                    analyze_citroen(b);
                    decode_data2 = (b[0] << 8) | b[1];
                    decode_data = 0;
                    for (int i = 0; i < 8; i++) decode_data = (decode_data << 8) | b[i + 2];
                    data_count_bit = 80;
                    if (callback) callback(this);
                    decode_data = 0;
                    decode_data2 = 0;
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif