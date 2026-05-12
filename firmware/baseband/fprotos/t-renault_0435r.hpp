#ifndef __FPROTO_TPMS_RENAULT_0435R_H__
#define __FPROTO_TPMS_RENAULT_0435R_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSRenault0435R : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSRenault0435R() {
        sensorType = FPT_Renault_0435R;
        te_short = 52;
        te_long = 104;
        te_delta = 35;
        min_count_bit_for_found = 72;  // Pontosan 9 bájt
    }

    bool checksum_renault(uint8_t* b) {
        uint8_t crc = 0;
        // Az 1-től 9-ig tartó bájtok (index 0-8) XOR összege pontosan 0 kell legyen
        for (int i = 0; i < 9; i++) {
            crc ^= b[i];
        }
        return (crc == 0);
    }

    void analyze_renault(uint8_t* b) {
        id = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
        pressure = (float)b[4] / 0.75f;
        temperature = (float)b[5] - 50.0f;
        battery = 100;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;
        uint32_t mid = (te_short + te_long) / 2;

        if (duration >= (te_short - te_delta) && duration <= mid) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (duration > mid && duration <= (te_long + te_delta)) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Zaj esetén nullázzuk a dekódert, az ablak ürül
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        bool bitstate;
        if (FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate)) {
            // Folyamatos 128-bites csúszóablak léptetése
            decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;

            if (decode_count_bit >= min_count_bit_for_found) {
                if (((decode_data >> 32) & 0xFF) != 0xC0) return;
                uint8_t b[9];
                uint8_t b_inv[9];

                // Vágjuk ki a legutolsó 72 bitet (9 bájt)
                b[0] = decode_data2 & 0xFF;
                b_inv[0] = ~b[0];

                for (int i = 0; i < 8; i++) {
                    b[i + 1] = (decode_data >> (56 - i * 8)) & 0xFF;
                    b_inv[i + 1] = ~b[i + 1];
                }

                // Tiszta, statikus zaj kiszűrése (ha nincs adat, csak üres éter)
                if ((b[0] == 0x00 && b[1] == 0x00) || (b[0] == 0xFF && b[1] == 0xFF)) return;

                bool match = false;
                uint8_t* valid_b = nullptr;

                // Teszteljük mindkét fázist (Normál és Invertált)
                if (checksum_renault(b)) {
                    match = true;
                    valid_b = b;
                } else if (checksum_renault(b_inv)) {
                    match = true;
                    valid_b = b_inv;
                }

                if (match) {
                    analyze_renault(valid_b);

                    // =========================================================
                    // DIAGNOSZTIKA: Ha a Checksum jó, de a Flag nem 0xC0,
                    // akkor rátettünk egy FF-et az ID elejére!
                    // Így biztosan tudni fogjuk, ha ezen akadt fent korábban.
                    // =========================================================
                    if (valid_b[3] != 0xC0) {
                        id = 0xFF000000 | id;
                    }

                    // Adatok előkészítése a UI-nak
                    decode_data2 = valid_b[0];
                    decode_data = 0;
                    for (int i = 0; i < 8; i++) decode_data = (decode_data << 8) | valid_b[i + 1];
                    data_count_bit = 72;

                    // KIÍRÁS A KÉPERNYŐRE!
                    if (callback) callback(this);

                    // Találat után nullázzuk a számlálókat a következő csomagig
                    decode_count_bit = 0;
                    decode_data = 0;
                    decode_data2 = 0;
                    FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif