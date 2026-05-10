#ifndef __FPROTO_TPMS_HYUNDAI_VDO_H__
#define __FPROTO_TPMS_HYUNDAI_VDO_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSHyundaiVDO : public FProtoSubTPMSBase {
   public:
    uint16_t sync_reg = 0;
    bool collecting_payload = false;
    int payload_bits = 0;
    bool inverted_phase = false;

    FProtoSubTPMSHyundaiVDO() {
        sensorType = FPT_HyundaiVDO;
        // A bizonyítottan tökéletes időzítések a te szenzorodhoz!
        te_short = 52;
        te_long = 104;
        te_delta = 25;
    }

    uint8_t crc8_vdo(uint8_t* data, size_t len) {
        uint8_t crc = 0xAA;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x07;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    void analyze_hyundai(uint8_t* b) {
        id = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];
        pressure = (float)b[6] * 1.375f;
        temperature = (float)b[7] - 50.0f;
        battery = b[8];
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Zaj vagy jelszakadás: AZONNALI NULLÁZÁS!
            // Ez védi meg a rendszert attól, hogy a zajt adatként kezelje.
            if (sync_reg > 0 || collecting_payload) {
                FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                sync_reg = 0;
                collecting_payload = false;
                payload_bits = 0;
            }
            return;
        }

        bool bitstate;
        if (FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate)) {
            if (!collecting_payload) {
                // 1. FÁZIS: Szinkronszó vadászat
                sync_reg = (sync_reg << 1) | bitstate;

                // Keresünk 9 nullát és 1 egyest (10 bites maszk: 0x03FF).
                // Mivel a szenzor valójában 15 nullát küld, hagyunk mozgásteret a rádiónak.
                if ((sync_reg & 0x03FF) == 0x0001) {
                    collecting_payload = true;
                    inverted_phase = false;
                    payload_bits = 0;
                    decode_data = 0;
                    decode_data2 = 0;
                } else if ((sync_reg & 0x03FF) == 0x03FE) {
                    // Invertált fázis: 9 egyes, 1 nulla
                    collecting_payload = true;
                    inverted_phase = true;
                    payload_bits = 0;
                    decode_data = 0;
                    decode_data2 = 0;
                }
            } else {
                // 2. FÁZIS: Pontosan 80 bit adat beolvasása
                decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
                decode_data = (decode_data << 1) | bitstate;
                payload_bits++;

                if (payload_bits == 80) {
                    uint8_t b[10];
                    b[0] = (decode_data2 >> 8) & 0xFF;
                    b[1] = (decode_data2) & 0xFF;
                    for (int i = 0; i < 8; i++) b[i + 2] = (decode_data >> (56 - i * 8)) & 0xFF;

                    if (inverted_phase) {
                        for (int i = 0; i < 10; i++) b[i] = ~b[i];
                    }

                    // Szigorú CRC ellenőrzés
                    if (crc8_vdo(b, 9) == b[9]) {
                        analyze_hyundai(b);
                        data_count_bit = 80;
                        if (callback) callback(this);
                    }

                    // Sikeres csomag (vagy hibás CRC) után azonnal visszatérünk a Szinkronszó vadászathoz!
                    collecting_payload = false;
                    sync_reg = 0;
                    payload_bits = 0;
                    FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif