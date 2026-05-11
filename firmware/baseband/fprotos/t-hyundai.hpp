#ifndef __FPROTO_TPMS_HYUNDAI_VDO_H__
#define __FPROTO_TPMS_HYUNDAI_VDO_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSHyundaiVDO : public FProtoSubTPMSBase {
   public:
    uint32_t decode_data_hi = 0; // A 96-bites ablak felső része
    uint32_t decode_data2 = 0;   // A UI-nak fenntartott változó

    FProtoSubTPMSHyundaiVDO() {
        sensorType = FPT_HyundaiVDO;
    }

    uint8_t crc8_vdo(uint8_t* data, size_t len) {
        uint8_t crc = 0xAA;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80) crc = (crc << 1) ^ 0x07;
                else crc <<= 1;
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

        // DINAMIKUS, HÉZAG NÉLKÜLI HATÁROK (Nincs elveszett impulzus!)
        // Az elméleti határok 52us és 104us. Félúton a határ pontosan 78us.
        if (duration >= 20 && duration <= 78) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (duration > 78 && duration <= 140) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Extrém zaj esetén nullázunk
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_data = 0;
            decode_data_hi = 0;
            return;
        }

        bool bitstate;
        if (FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate)) {
            
            // 96-BITES CSÚSZÓABLAK (16 bit szinkron + 80 bit adat)
            decode_data_hi = (decode_data_hi << 1) | ((decode_data >> 63) & 1);
            decode_data = (decode_data << 1) | bitstate;

            // Keresünk 11 nullát és 1 egyest (0x0FFF maszk) az ablak legtetején
            uint16_t pre12 = (decode_data_hi >> 16) & 0x0FFF;

            if (pre12 == 0x0001 || pre12 == 0x0FFE) {
                uint8_t b[10];
                // A payload kivágása a 96-bites ablakból
                b[0] = (decode_data_hi >> 8) & 0xFF;
                b[1] = (decode_data_hi) & 0xFF;
                for (int i = 0; i < 8; i++) b[i + 2] = (decode_data >> (56 - i * 8)) & 0xFF;

                // Fázisfordítás ellenőrzése
                if (pre12 == 0x0FFE) {
                    for (int i = 0; i < 10; i++) b[i] = ~b[i];
                }

                // Végső CRC szűrő
                if (crc8_vdo(b, 9) == b[9]) {
                    analyze_hyundai(b);
                    
                    // A UI-hoz (képernyőhöz) szükséges formátum előkészítése
                    decode_data2 = (b[0] << 8) | b[1];
                    decode_data = 0;
                    for (int i = 0; i < 8; i++) decode_data = (decode_data << 8) | b[i + 2];
                    data_count_bit = 80;

                    // KIÍRÁS A KÉPERNYŐRE!
                    if (callback) callback(this);

                    // Sikeres csomag után nullázzuk az ablakot, hogy ne duplázzon
                    decode_data = 0;
                    decode_data_hi = 0;
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif