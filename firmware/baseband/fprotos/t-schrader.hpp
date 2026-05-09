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
        // 1. Szigorú szűrő: Az rtl_433 szerint a Schrader EG53MA4 payload
        // MINDIG 0x4-el kezdődik. Ez kizárja az összes "szellem" csomagot.
        if ((b[0] & 0xF0) != 0x40) return false;

        // 2. Modulo-256 Checksum ellenőrzése
        uint8_t sum = 0;
        for (int i = 0; i < 9; i++) {
            sum += b[i];
        }
        if (sum != b[9]) return false;

        // 3. Halott, csupa 0 vagy csupa FF csomagok kiszűrése
        if (!b[1] && !b[2] && !b[4] && !b[5] && !b[7] && !b[8]) return false;
        if (b[4] == 0x00 && b[5] == 0x00 && b[6] == 0x00) return false;
        if (b[4] == 0xFF && b[5] == 0xFF && b[6] == 0xFF) return false;

        return true;
    }

    void analyze_eg53ma4(uint8_t* b) {
        // ID: 24 bit
        id = (b[4] << 16) | (b[5] << 8) | b[6];

        // Nyomás: az eddigi adatok alapján az rtl_433 a * 2.5f szorzót használja
        pressure = (float)b[7] * 2.5f;

        // Hőmérséklet: Fahrenheitből Celsiusba váltás
        temperature = ((float)b[8] - 32.0f) * (5.0f / 9.0f);

        battery = 0xFF;  // Nincs dedikált akku állapot
    }

    void feed(bool level, uint32_t duration) {
        // 1. SZÜNET ÉRZÉKELŐ (Timeout, a csomag vége)
        if (level == false && duration > 400) {
            // Ha van elegendő bitünk a kiolvasáshoz
            if (decode_count_bit >= 80) {
                bool found = false;

                // Ablakos szkennelés az esetleges szinkronizációs elcsúszások miatt
                for (int offset = 0; offset <= 16 && !found; offset++) {
                    uint8_t b[10];
                    uint8_t b_inv[10];

                    // Bitek kinyerése a csúszó ablak alapján
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

                    // Invertált fázis elkészítése
                    for (int i = 0; i < 10; i++) b_inv[i] = ~b[i];

                    // ELŐSZÖR AZ INVERTÁLTAT VIZSGÁLJUK MEG
                    if (sanity_check_eg53ma4(b_inv)) {
                        data_count_bit = 80;

                        // Visszaírjuk a letisztított, tökéletes adatot a UI debug számára!
                        decode_data2 = (b_inv[0] << 8) | b_inv[1];
                        decode_data = 0;
                        for (int i = 0; i < 8; i++) {
                            decode_data = (decode_data << 8) | b_inv[i + 2];
                        }

                        analyze_eg53ma4(b_inv);
                        if (callback) callback(this);
                        found = true;
                    }
                    // HA OTT NINCS, MEGNÉZZÜK A NORMÁL FÁZIST
                    else if (sanity_check_eg53ma4(b)) {
                        data_count_bit = 80;

                        // Visszaírjuk a letisztított, tökéletes adatot a UI debug számára!
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

            // Takarítás a következő csomag érkezéséhez
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        // 2. TIMING CLASSIFIER (Impulzusok hossza alapján)
        ManchesterEvent event = ManchesterEventReset;
        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Ha zaj érkezik, azonnal eldobunk mindent és tiszta lappal indulunk
            if (decode_count_bit > 0) {
                FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                decode_count_bit = 0;
                decode_data = 0;
                decode_data2 = 0;
            }
            return;
        }

        // 3. MANCHESTER DECODER (Bitek beolvasása)
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