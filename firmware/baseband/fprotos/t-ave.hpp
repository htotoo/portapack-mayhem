#ifndef __FPROTO_TPMS_AVE_H__
#define __FPROTO_TPMS_AVE_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSAVE : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSAVE() {
        sensorType = FPT_AVE;
        te_short = 100;
        te_long = 200;
        te_delta = 40;
        min_count_bit_for_found = 64;
    }

    void tpms_protocol_ave_analyze(uint8_t* b) {
        id = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
        int pressure_raw = b[4];
        int temperature_raw = b[5];
        int mode = (b[6] >> 6) & 0x03;
        int battery_raw = (b[6] >> 3) & 0x07;

        if (battery_raw == 7)
            battery = 25;
        else if (battery_raw == 6)
            battery = 75;
        else
            battery = 100;

        temperature = temperature_raw - 50;

        float ratio = 2.352f, offset = 0.0f;
        switch (mode) {
            case 0:
                ratio = 2.352f;
                offset = 47.0f;
                break;
            case 1:
                ratio = 2.352f;
                offset = 0.0f;
                break;
            case 2:
                ratio = 5.491f;
                offset = 18.2f;
                break;
            case 3:
                ratio = 5.491f;
                offset = 0.0f;
                break;
        }

        pressure = (((float)pressure_raw - offset) * ratio) * 0.01f;
    }

    void feed(bool level, uint32_t duration) {
        (void)level;
        bool bitstate, data_ok = false;

        if (DURATION_DIFF(duration, te_short) < te_delta) {
            if (prev_short) {
                bitstate = 0;
                data_ok = true;
                prev_short = false;
            } else {
                prev_short = true;
            }
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            if (prev_short) {
                decode_count_bit = 0;
                decode_data = 0;
                prev_short = false;
                return;
            }
            bitstate = 1;
            data_ok = true;
        } else {
            decode_count_bit = 0;
            decode_data = 0;
            prev_short = false;
            return;
        }

        if (data_ok) {
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;

            if (decode_count_bit >= min_count_bit_for_found) {
                // ==========================================
                // AVE FAST EXIT
                // ==========================================
                if (decode_data == 0ULL || decode_data == ~0ULL) return;  // Nyers zaj azonnali dobása

                uint8_t b[8];
                for (int i = 0; i < 8; i++) {
                    b[i] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                }

                if (FProtoGeneral::subghz_protocol_blocks_crc8(b, 8, 0x31, 0xFF) == 0x00) {
                    data_count_bit = 64;
                    tpms_protocol_ave_analyze(b);
                    if (callback) callback(this);
                    decode_count_bit = 0;
                    decode_data = 0;
                    prev_short = false;
                }
            }
        }
    }

   protected:
    bool prev_short = false;
};

#endif