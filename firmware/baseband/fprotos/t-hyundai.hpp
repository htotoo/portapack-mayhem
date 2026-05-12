#ifndef __FPROTO_TPMS_VDO_COMBINED_H__
#define __FPROTO_TPMS_VDO_COMBINED_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMS_VDO : public FProtoSubTPMSBase {
   public:
    uint32_t decode_data_hi = 0;
    uint32_t decode_data2 = 0;

    FProtoSubTPMS_VDO() {
        sensorType = FPT_HyundaiVDO;  // Alapértelmezett, de dinamikusan cseréljük!
        te_short = 52;
        te_long = 104;
        te_delta = 35;
    }

    uint8_t crc8_hyundai(uint8_t* data, size_t len) {
        uint8_t crc = 0xAA;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
        return crc;
    }

    bool checksum_citroen(uint8_t* b) {
        uint8_t crc = 0;
        for (int i = 1; i <= 9; i++) crc ^= b[i];
        return (crc == 0 && b[6] != 0 && b[7] != 0);  // Sanity check is beépítve
    }

    void analyze_payload(uint8_t* b, int type) {
        if (type == 1) {  // Citroen
            id = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];
            pressure = (float)b[6] * 1.364f;
            temperature = (float)b[7] - 50.0f;
            battery = b[8];
            sensorType = FPT_Citroen;  // Dinamikus átváltás a UI-nak
        } else {                       // Hyundai
            id = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];
            pressure = (float)b[6] * 1.375f;
            temperature = (float)b[7] - 50.0f;
            battery = b[8];
            sensorType = FPT_HyundaiVDO;
        }
    }

    void feed(bool level, uint32_t duration) {
        // Gyors zaj-zsilip (Glitch filter)
        if (duration < 15) return;

        ManchesterEvent event = ManchesterEventReset;
        uint32_t mid = (te_short + te_long) / 2;

        if (duration >= (te_short - te_delta) && duration <= mid) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (duration > mid && duration <= (te_long + te_delta)) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_data = 0;
            decode_data_hi = 0;
            return;
        }

        bool bitstate;
        if (FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate)) {
            decode_data_hi = (decode_data_hi << 1) | ((decode_data >> 63) & 1);
            decode_data = (decode_data << 1) | bitstate;

            uint16_t pre12 = (decode_data_hi >> 16) & 0x0FFF;

            // Csak akkor megyünk tovább, ha megvan a közös VDO preambulum
            if (pre12 == 0x0001 || pre12 == 0x0FFE) {
                // Tiszta zaj kiszűrése array építés előtt
                if (decode_data == 0ULL || decode_data == ~0ULL) return;

                uint8_t b[10];
                b[0] = (decode_data_hi >> 8) & 0xFF;
                b[1] = (decode_data_hi) & 0xFF;
                for (int i = 0; i < 8; i++) b[i + 2] = (decode_data >> (56 - i * 8)) & 0xFF;

                if (pre12 == 0x0FFE) {
                    for (int i = 0; i < 10; i++) b[i] = ~b[i];
                }

                // 1. Próba: Ez egy Citroen?
                if (checksum_citroen(b)) {
                    analyze_payload(b, 1);
                    goto packet_found;
                }
                // 2. Próba: Ez egy Hyundai?
                else if (crc8_hyundai(b, 9) == b[9]) {
                    analyze_payload(b, 2);
                    goto packet_found;
                }

                return;  // Ha egyik sem volt, kilépünk.

            packet_found:
                decode_data2 = (b[0] << 8) | b[1];
                decode_data = 0;
                for (int i = 0; i < 8; i++) decode_data = (decode_data << 8) | b[i + 2];
                data_count_bit = 80;
                if (callback) callback(this);
                decode_data = 0;
                decode_data_hi = 0;
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif