#ifndef __FPROTO_TPMS_SCHRADER_H__
#define __FPROTO_TPMS_SCHRADER_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSSchraderEG53MA4 : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSSchraderEG53MA4() {
        sensorType = FPT_Schrader_EG53MA4;
        te_short = 120;
        te_long = 240;
        te_delta = 60;
    }

    bool sanity_check_eg53ma4(uint8_t* b) {
        uint8_t sum = 0;
        for (int i = 0; i < 9; i++) {
            sum += b[i];
        }
        if (sum != b[9]) return false;
        if (!b[1] && !b[2] && !b[4] && !b[5] && !b[7] && !b[8]) return false;
        if (b[4] == 0x00 && b[5] == 0x00 && b[6] == 0x00) return false;
        if (b[4] == 0xFF && b[5] == 0xFF && b[6] == 0xFF) return false;
        return true;
    }

    void analyze_eg53ma4(uint8_t* b) {
        id = (b[4] << 16) | (b[5] << 8) | b[6];
        pressure = (float)b[7] * 2.5f;                        // kpa
        temperature = ((float)b[8] - 32.0f) * (5.0f / 9.0f);  // celsius
        battery = 0xFF;                                       // no battery info
    }

    void feed(bool level, uint32_t duration) {
        if (level == false && duration > 400) {
            if (decode_count_bit >= 100) {
                bool found = false;
                for (int offset = 0; offset <= 16 && !found; offset++) {
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
                    uint8_t preamble_raw = (d2 >> 16) & 0xFF;

                    if (preamble_raw == 0xFF && sanity_check_eg53ma4(b_inv)) {
                        data_count_bit = 80;
                        decode_data2 = (b_inv[0] << 8) | b_inv[1];
                        decode_data = 0;
                        for (int i = 0; i < 8; i++) {
                            decode_data = (decode_data << 8) | b_inv[i + 2];
                        }
                        analyze_eg53ma4(b_inv);
                        if (callback) callback(this);
                        found = true;
                    } else if (preamble_raw == 0x00 && sanity_check_eg53ma4(b)) {
                        data_count_bit = 80;
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

            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

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

// ==============================================================================
// 2. A Klasszikus Schrader TPMS (68-bit) CRC8 ellenőrzéssel
// ==============================================================================
class FProtoSubTPMSSchrader : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSSchrader() {
        sensorType = FPT_Schrader;
        te_short = 120;
        te_long = 240;
        te_delta = 60;
    }

    uint8_t crc8_schrader(uint8_t* data, size_t len) {
        uint8_t crc = 0xF0;
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

    bool sanity_check_schrader(uint8_t* b) {
        if (b[7] != crc8_schrader(b, 7)) return false;
        if (!b[1] && !b[2] && !b[3] && !b[4] && !b[5] && !b[6]) return false;
        return true;
    }

    void analyze_schrader(uint8_t* b) {
        id = ((b[1] & 0x0F) << 24) | (b[2] << 16) | (b[3] << 8) | b[4];
        pressure = (float)b[5] * 2.5f;
        temperature = (float)b[6] - 50.0f;
        battery = 0xFF;
    }

    void feed(bool level, uint32_t duration) {
        if (level == false && duration > 400) {
            // Csomag 68 bit, de a hasznos payload 64 bit a végén.
            if (decode_count_bit >= 68) {
                bool found = false;
                for (int offset = 0; offset <= 16 && !found; offset++) {
                    uint8_t b[8];
                    uint8_t b_inv[8];

                    uint64_t d1 = decode_data >> offset;
                    if (offset > 0) {
                        uint64_t mask = (1ULL << offset) - 1;
                        d1 |= (decode_data2 & mask) << (64 - offset);
                    }

                    for (int i = 0; i < 8; i++) {
                        b[i] = (d1 >> (56 - i * 8)) & 0xFF;
                        b_inv[i] = ~b[i];
                    }

                    if (sanity_check_schrader(b_inv)) {
                        data_count_bit = 64;
                        decode_data = 0;
                        for (int i = 0; i < 8; i++) decode_data = (decode_data << 8) | b_inv[i];
                        analyze_schrader(b_inv);
                        if (callback) callback(this);
                        found = true;
                    } else if (sanity_check_schrader(b)) {
                        data_count_bit = 64;
                        decode_data = 0;
                        for (int i = 0; i < 8; i++) decode_data = (decode_data << 8) | b[i];
                        analyze_schrader(b);
                        if (callback) callback(this);
                        found = true;
                    }
                }
            }
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

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

// ==============================================================================
// 3. Schrader SMD3MA4 (PCM 110-bit) Subaru/Nissan protokoll
// ==============================================================================
class FProtoSubTPMSSchraderSMD3MA4 : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSSchraderSMD3MA4() {
        sensorType = FPT_Schrader;
        te_short = 120;
        te_long = 240;
        te_delta = 60;
    }

    void analyze_smd3ma4(uint64_t data37) {
        id = (data37 >> 10) & 0xFFFFFF;
        int raw_pressure = (data37 >> 2) & 0xFF;

        // Az rtl_433 25.2 PSI-t mutatott.
        // A Portapack UI szabványosan kPa-t vár, ezért a 25.2 PSI-t átváltjuk:
        // 25.2 * 6.89476 = 173.7 kPa. A kijelződön ezt fogod látni!
        pressure = (float)raw_pressure * 0.2f * 6.89476f;
        temperature = 0;  // Ebben a protokollban nincs hőmérséklet
        battery = 0xFF;
    }

    void feed(bool level, uint32_t duration) {
        // 1. SZÜNET ÉRZÉKELŐ ÉS FELDOLGOZÓ
        if (level == false && duration > 600) {
            // "Záró Nullák" pótlása: A csomag végén lévő csendes nullákat
            // betoljuk a PCM pufferbe, hogy a csomag garantáltan teljes legyen.
            for (int i = 0; i < 4; i++) {
                decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
                decode_data = (decode_data << 1) | 0;
                if (decode_count_bit < 128) decode_count_bit++;
            }

            if (decode_count_bit >= 110) {
                bool found = false;

                // Keressük a Preamble-t a csúszóablakban
                for (int offset = 0; (int32_t)offset <= (int32_t)decode_count_bit - 110 && !found; offset++) {
                    // A 36-bites preamble (0xF5555555E) kinyerése
                    uint64_t preamble = (decode_data2 >> (offset + 10)) & 0xFFFFFFFFFULL;

                    // Ellenőrizzük a normál ÉS az SDR által invertált fázist is
                    if (preamble == 0xF5555555EULL || preamble == 0x0AAAAAAA1ULL) {
                        bool inverted = (preamble == 0x0AAAAAAA1ULL);

                        // Kinyerjük a Preamble utáni 74 bites nyers PCM adatot
                        uint64_t pcm_lo = decode_data;
                        uint64_t pcm_hi = decode_data2;
                        if (offset > 0) {
                            pcm_lo = (decode_data >> offset) | (decode_data2 << (64 - offset));
                            pcm_hi = decode_data2 >> offset;
                        }
                        pcm_hi &= 0x3FF;  // Csak a felső 10 bit kell a 74-hez

                        if (inverted) {
                            pcm_lo = ~pcm_lo;
                            pcm_hi = (~pcm_hi) & 0x3FF;
                        }

                        // Saját, szoftveres Manchester dekódolás (74 nyers bit -> 37 adat bit)
                        uint64_t data37 = 0;
                        bool manchester_ok = true;

                        for (int i = 0; i < 37; i++) {
                            int bit_idx = 72 - (i * 2);
                            uint8_t pair;

                            // 2 bites Manchester párok kivágása a 64 bites határokon átnyúlva
                            if (bit_idx >= 64) {
                                pair = (pcm_hi >> (bit_idx - 64)) & 3;
                            } else if (bit_idx == 63) {
                                pair = ((pcm_hi & 1) << 1) | (pcm_lo >> 63);
                            } else {
                                pair = (pcm_lo >> bit_idx) & 3;
                            }

                            // Az rtl_433 inverz dekódolást használ: 01 => 0, 10 => 1
                            if (pair == 1) {
                                data37 = (data37 << 1) | 0;
                            } else if (pair == 2) {
                                data37 = (data37 << 1) | 1;
                            } else {
                                manchester_ok = false;  // Illegális Manchester pár
                                break;
                            }
                        }

                        if (manchester_ok) {
                            data_count_bit = 37;
                            decode_data = data37;  // Nyers 37 bites adat megy a UI debug sorba
                            decode_data2 = 0;
                            analyze_smd3ma4(data37);
                            if (callback) callback(this);
                            found = true;
                        }
                    }
                }
            }

            // Takarítás a következő csomaghoz
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        // 2. NYERS PCM SZELETELŐ (A manchester_advance helyett!)
        // Kiszámoljuk, hány darab 120µs-os bit fér bele a kapott impulzusba
        int pcm_bits = (duration + te_delta) / te_short;

        // Ha túl rövid (zaj) vagy túl hosszú, reseteljük a puffert
        if (pcm_bits == 0 || pcm_bits > 5) {
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        // Betoljuk a nyers feszültségszintet (level) annyiszor, ahány bites volt az impulzus
        for (int i = 0; i < pcm_bits; i++) {
            decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
            decode_data = (decode_data << 1) | level;

            if (decode_count_bit < 128) {
                decode_count_bit++;
            }
        }
    }
};
#endif