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

    void analyze_eg53ma4(uint8_t* b) {
        id = (b[4] << 16) | (b[5] << 8) | b[6];
        pressure = (float)b[7] * 2.5f;
        temperature = ((float)b[8] - 32.0f) * (5.0f / 9.0f);
        battery = 0xFF;
    }

    void feed(bool level, uint32_t duration) {
        // 1. SZÜNET ÉRZÉKELŐ (Timeout)
        if (level == false && duration > 400) {
            // Ha van elég bitünk, elindítjuk a "Vadász" szkennert
            if (decode_count_bit >= 80) {
                bool found = false;

                // Végignézzük az összes lehetséges csúszást a 128 bites pufferben
                for (int offset = 0; offset <= 48 && !found; offset++) {
                    uint8_t b[10];
                    uint8_t b_inv[10];

                    uint64_t d1 = decode_data >> offset;
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

                    // CHECKSUM HELYETT A FIX EMULÁTOR ID-t KERESSÜK! (34 56 78)
                    uint32_t current_id = (b[4] << 16) | (b[5] << 8) | b[6];
                    uint32_t current_id_inv = (b_inv[4] << 16) | (b_inv[5] << 8) | b_inv[6];

                    // Ha megvan az ID, rögzítjük az eltolást és kiolvassuk a nyomást
                    if (current_id == 0x345678) {
                        analyze_eg53ma4(b);
                        if (callback) callback(this);
                        found = true;
                    } else if (current_id_inv == 0x345678) {
                        analyze_eg53ma4(b_inv);
                        if (callback) callback(this);
                        found = true;
                    }
                }
            }

            // Takarítás
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        // 2. NORMÁL FELDOLGOZÁS
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