#ifndef __FPROTO_TPMS_TOYOTA_H__
#define __FPROTO_TPMS_TOYOTA_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSToyota : public FProtoSubTPMSBase {
   public:
    uint32_t decode_data2 = 0;

    // Chip-alapú állapotgép változói
    uint8_t last_chip = 2;   // Az előző chip szintje (2 = inicializálatlan)
    uint8_t chip_phase = 0;  // 0 = bit határa, 1 = bit közepe

    FProtoSubTPMSToyota() {
        sensorType = FPT_Toyota;
        te_short = 52;
        te_long = 104;
        te_delta = 25;
        min_count_bit_for_found = 72;  // 9 bájt payload
    }

    void tpms_protocol_toyota_analyze(uint8_t* b) {
        id = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];

        float psi = (((b[4] & 0x7F) << 1) | (b[5] >> 7)) * 0.25f - 7.0f;  //
        pressure = psi * 6.89476f;

        int temp = ((b[5] & 0x7F) << 1) | (b[6] >> 7);  //
        temperature = (float)temp - 40.0f;              //

        battery = 100;
    }

    bool checksum_toyota(uint8_t* b) {
        uint8_t crc = 0x80;  // A Toyota CRC-8 inicializálása 0x80
        for (int i = 0; i < 8; i++) {
            crc ^= b[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x07;
                else
                    crc <<= 1;
            }
        }
        return (crc == b[8]);  //
    }

    bool toyota_payload_sanity_check(uint8_t* b) {
        // Nyomás-Integritás szűrő: nyomás összevetése az invertált párjával
        uint8_t p1 = ((b[4] & 0x7F) << 1) | (b[5] >> 7);  //
        uint8_t p2 = b[7] ^ 0xFF;                         //
        if (p1 != p2) return false;                       //

        return checksum_toyota(b);  //
    }

    void feed(bool level, uint32_t duration) {
        int chips = 0;

        // Impulzus felbontása Chipekre (Fél-bitekre)
        if (duration >= (te_short - te_delta) && duration <= (te_short + te_delta)) {
            chips = 1;
        } else if (duration >= (te_long - te_delta) && duration <= (te_long + te_delta)) {
            chips = 2;  // Egy hosszú impulzusban KÉT azonos szintű chip van!
        } else {
            // Toyota hatalmas Preamble jele vagy zaj -> Ablak ürítése és fázis szinkronizálása
            chip_phase = 0;
            last_chip = level;
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        // Chip-alapú Differenciális Manchester Dekóder
        for (int i = 0; i < chips; i++) {
            if (chip_phase == 0) {
                // Bit határ: Ha nincs élváltás (szint egyezik) -> 1-es bit. Ha van -> 0-ás bit.
                uint8_t bit = (level == last_chip) ? 1 : 0;

                // Csúszóablak léptetése
                decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
                decode_data = (decode_data << 1) | bit;
                decode_count_bit++;

                chip_phase = 1;  // A következő chip a bit közepe lesz
            } else {
                // Bit közepe: KÖTELEZŐ az élváltás
                if (level == last_chip) {
                    // HIBA! Fázistörés történt. Öngyógyítás: ez valószínűleg egy új bit határa.
                    chip_phase = 1;
                    decode_count_bit = 0;  // Ablak ürítése
                } else {
                    // Tökéletes élváltás, a következő chip újra bit-határ lesz
                    chip_phase = 0;
                }
            }
            last_chip = level;  // Eltároljuk a fizikai szintet
        }

        // Ha megvan a 72 bit, jöhet a validálás
        if (decode_count_bit >= min_count_bit_for_found) {
            uint8_t p1 = ((((decode_data >> 24) & 0x7F) << 1) | (((decode_data >> 16) & 0xFF) >> 7));
            uint8_t p2 = (decode_data & 0xFF) ^ 0xFF;

            if (p1 != p2) return;
            uint8_t b[9], b_inv[9];

            b[0] = decode_data2 & 0xFF;
            b_inv[0] = ~b[0];
            for (int i = 0; i < 8; i++) {
                b[i + 1] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                b_inv[i + 1] = ~b[i + 1];
            }

            bool match = false;
            uint8_t* valid_b = nullptr;

            if (toyota_payload_sanity_check(b)) {
                match = true;
                valid_b = b;
            } else if (toyota_payload_sanity_check(b_inv)) {
                match = true;
                valid_b = b_inv;
            }

            if (match) {
                tpms_protocol_toyota_analyze(valid_b);

                // UI adat formázása
                decode_data2 = valid_b[0];
                decode_data = 0;
                for (int i = 0; i < 8; i++) decode_data = (decode_data << 8) | valid_b[i + 1];
                data_count_bit = 72;

                if (callback) callback(this);

                // Sikeres olvasás után ürítünk
                decode_count_bit = 0;
                decode_data = 0;
                decode_data2 = 0;
            }
        }
    }
};

#endif