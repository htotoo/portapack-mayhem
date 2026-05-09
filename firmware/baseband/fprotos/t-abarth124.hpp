#ifndef __FPROTO_TPMS_ABARTH124_H__
#define __FPROTO_TPMS_ABARTH124_H__

#include "subtpmsbase.hpp"

#define MODEL_TG1C 1
#define MODEL_Q85 2

typedef enum {
    AbarthDecoderStepSearch = 0,
    AbarthDecoderStepPayload
} AbarthDecoderStep;

class FProtoSubTPMSAbarth124 : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSAbarth124() {
        sensorType = FPT_Abarth124;  // Ensure this enum exists in your global definitions

        te_short = 52;
        te_long = 104;
        te_delta = 25;
        min_count_bit_for_found = 72;  // Wait for gap to evaluate
    }

    uint8_t xor_bytes(uint8_t* data, int len) {
        uint8_t sum = 0;
        for (int i = 0; i < len; i++) sum ^= data[i];
        return sum;
    }

    // CRC-16 CCITT-FALSE (Poly 0x1021, Init 0xFFFF)
    uint16_t crc16_ccitt_false(uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    void tpms_protocol_abarth124_analyze() {
        uint8_t b[12] = {0};

        if (saved_type == MODEL_TG1C) {
            b[0] = (saved_data2) & 0xFF;
            b[1] = (saved_data >> 56) & 0xFF;
            b[2] = (saved_data >> 48) & 0xFF;
            b[3] = (saved_data >> 40) & 0xFF;
            b[4] = (saved_data >> 32) & 0xFF;
            b[5] = (saved_data >> 24) & 0xFF;
            b[6] = (saved_data >> 16) & 0xFF;
            b[7] = (saved_data >> 8) & 0xFF;
            b[8] = (saved_data) & 0xFF;
            if (saved_inverted)
                for (int i = 0; i < 9; i++) b[i] = ~b[i];

            data_count_bit = 72;
            sensorType = FPT_Abarth124;
        } else if (saved_type == MODEL_Q85) {
            b[0] = (saved_data2 >> 24) & 0xFF;
            b[1] = (saved_data2 >> 16) & 0xFF;
            b[2] = (saved_data2 >> 8) & 0xFF;
            b[3] = (saved_data2) & 0xFF;
            b[4] = (saved_data >> 56) & 0xFF;
            b[5] = (saved_data >> 48) & 0xFF;
            b[6] = (saved_data >> 40) & 0xFF;
            b[7] = (saved_data >> 32) & 0xFF;
            b[8] = (saved_data >> 24) & 0xFF;
            b[9] = (saved_data >> 16) & 0xFF;
            b[10] = (saved_data >> 8) & 0xFF;
            b[11] = (saved_data) & 0xFF;
            if (saved_inverted)
                for (int i = 0; i < 12; i++) b[i] = ~b[i];

            data_count_bit = 96;
            sensorType = FPT_Q85;
        }

        id = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
        battery = 0xFF;

        int press_raw = b[5];
        int temp_raw = b[6];

        if (saved_type == MODEL_TG1C) {
            pressure = (float)press_raw * 0.0138f;
            temperature = (float)temp_raw - 50.0f;
        } else if (saved_type == MODEL_Q85) {
            pressure = (float)press_raw * 0.03f;
            temperature = (float)temp_raw - 55.0f;
        }
    }

    bool sanity_check_abarth(uint8_t* b) {
        // ID should not be totally empty or totally saturated
        if (b[0] == 0x00 && b[1] == 0x00 && b[2] == 0x00 && b[3] == 0x00) return false;
        if (b[0] == 0xFF && b[1] == 0xFF && b[2] == 0xFF && b[3] == 0xFF) return false;

        // Pressure and temp rarely sit at exactly 0x00 or 0xFF at the same time
        if (b[5] == 0x00 && b[6] == 0x00) return false;
        if (b[5] == 0xFF && b[6] == 0xFF) return false;

        return true;
    }

    void reset_decoder() {
        FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
        parser_step = AbarthDecoderStepSearch;
        sync_reg = 0;
        decode_count_bit = 0;
        decode_data = 0;
        decode_data2 = 0;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Sequence gap or noise -> Evaluate what we have, then hard reset
            if (packet_ready) {
                tpms_protocol_abarth124_analyze();
                if (callback) callback(this);
                packet_ready = false;
            }
            reset_decoder();
            return;
        }

        bool bitstate;
        bool data_ok = FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate);

        if (data_ok) {
            if (parser_step == AbarthDecoderStepSearch) {
                sync_reg = (sync_reg << 1) | bitstate;
                uint32_t sync_word = sync_reg & 0xFFFFFF;  // Look at last 24 bits

                // Abarth / Q85 preamble is 0x555556 or 0xAAAAA9 (inverted FSK phase)
                if (sync_word == 0x555556 || sync_word == 0xAAAAA9) {
                    parser_step = AbarthDecoderStepPayload;
                    decode_count_bit = 0;
                    decode_data = 0;
                    decode_data2 = 0;
                    phase_inverted = (sync_word == 0xAAAAA9);
                }
            } else if (parser_step == AbarthDecoderStepPayload) {
                // Shift bit into 128-bit cascading window
                decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
                decode_data = (decode_data << 1) | bitstate;
                decode_count_bit++;

                // Strictly evaluate exactly at the byte boundaries
                if (decode_count_bit == 72) {
                    uint8_t b[9];
                    b[0] = (decode_data2) & 0xFF;
                    b[1] = (decode_data >> 56) & 0xFF;
                    b[2] = (decode_data >> 48) & 0xFF;
                    b[3] = (decode_data >> 40) & 0xFF;
                    b[4] = (decode_data >> 32) & 0xFF;
                    b[5] = (decode_data >> 24) & 0xFF;
                    b[6] = (decode_data >> 16) & 0xFF;
                    b[7] = (decode_data >> 8) & 0xFF;
                    b[8] = (decode_data) & 0xFF;

                    if (phase_inverted)
                        for (int i = 0; i < 9; i++) b[i] = ~b[i];

                    if (sanity_check_abarth(b) && xor_bytes(b, 9) == 0) {
                        saved_data = decode_data;
                        saved_data2 = decode_data2;
                        saved_type = MODEL_TG1C;
                        saved_inverted = phase_inverted;
                        packet_ready = true;
                    }
                } else if (decode_count_bit == 96) {
                    uint8_t b[12];
                    b[0] = (decode_data2 >> 24) & 0xFF;
                    b[1] = (decode_data2 >> 16) & 0xFF;
                    b[2] = (decode_data2 >> 8) & 0xFF;
                    b[3] = (decode_data2) & 0xFF;
                    b[4] = (decode_data >> 56) & 0xFF;
                    b[5] = (decode_data >> 48) & 0xFF;
                    b[6] = (decode_data >> 40) & 0xFF;
                    b[7] = (decode_data >> 32) & 0xFF;
                    b[8] = (decode_data >> 24) & 0xFF;
                    b[9] = (decode_data >> 16) & 0xFF;
                    b[10] = (decode_data >> 8) & 0xFF;
                    b[11] = (decode_data) & 0xFF;

                    if (phase_inverted)
                        for (int i = 0; i < 12; i++) b[i] = ~b[i];

                    uint16_t crc_little = (b[11] << 8) | b[10];
                    if (sanity_check_abarth(b) && xor_bytes(b, 9) == 0 && crc16_ccitt_false(b, 10) == crc_little) {
                        saved_data = decode_data;
                        saved_data2 = decode_data2;
                        saved_type = MODEL_Q85;  // Safely upgrades/overwrites a TG1C match
                        saved_inverted = phase_inverted;
                        packet_ready = true;
                    }
                } else if (decode_count_bit > 100) {
                    // Max length reached, flush and evaluate early
                    if (packet_ready) {
                        tpms_protocol_abarth124_analyze();
                        if (callback) callback(this);
                        packet_ready = false;
                    }
                    reset_decoder();
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
    AbarthDecoderStep parser_step = AbarthDecoderStepSearch;
    uint32_t sync_reg = 0;
    bool phase_inverted = false;

    // Gap-evaluation variables
    uint64_t saved_data = 0;
    uint64_t saved_data2 = 0;
    int saved_type = 0;
    bool saved_inverted = false;
    bool packet_ready = false;
};

#endif