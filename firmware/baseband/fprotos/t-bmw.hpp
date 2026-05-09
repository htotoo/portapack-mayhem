#ifndef __FPROTO_TPMS_BMW_H__
#define __FPROTO_TPMS_BMW_H__

#include "subtpmsbase.hpp"

#define MODEL_AUDI 1
#define MODEL_BMW 2

class FProtoSubTPMSBMW : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSBMW() {
        sensorType = FPT_BMW;  // Ensure this enum exists in your global definitions

        // rtl_433 defines short/long width as 25us for this FSK PCM.
        // This is a very fast bit-rate!
        te_short = 25;
        te_long = 50;
        te_delta = 12;                 // Tighten jitter margin for faster pulses
        min_count_bit_for_found = 64;  // Minimum length is 8 bytes (Audi Alert)
    }

    void tpms_protocol_bmw_analyze() {
        uint8_t b[11] = {0};

        // Unpack the saved data blocks back into a working byte array
        if (saved_type == MODEL_AUDI) {
            for (int i = 0; i < 8; i++) {
                b[i] = (saved_data >> ((7 - i) * 8)) & 0xFF;
            }
            sensorType = FPT_AUDI;
        } else if (saved_type == MODEL_BMW) {
            b[0] = (saved_data2 >> 16) & 0xFF;
            b[1] = (saved_data2 >> 8) & 0xFF;
            b[2] = (saved_data2) & 0xFF;
            b[3] = (saved_data >> 56) & 0xFF;
            b[4] = (saved_data >> 48) & 0xFF;
            b[5] = (saved_data >> 40) & 0xFF;
            b[6] = (saved_data >> 32) & 0xFF;
            b[7] = (saved_data >> 24) & 0xFF;
            b[8] = (saved_data >> 16) & 0xFF;
            b[9] = (saved_data >> 8) & 0xFF;
            b[10] = (saved_data) & 0xFF;
            sensorType = FPT_BMW;
        }

        // I = Sensor ID (Bytes 1 to 4)
        id = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];

        // P = Pressure (Byte 5)
        // Formula is b[5] * 2.45 = kPa
        // Portapack expects Bar (1 Bar = 100 kPa) -> multiply by 0.0245
        pressure = (float)b[5] * 0.0245f;

        // T = Temperature (Byte 6, offset by 52)
        temperature = (float)b[6] - 52.0f;

        // Battery status is not definitively mapped (could be buried in flags F1/F2/F3)
        battery = 0xFF;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        // Evaluate the pulse timing
        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Sequence break / gap. Evaluate if we successfully caught a packet.
            if (packet_ready) {
                data_count_bit = (saved_type == MODEL_BMW) ? 88 : 64;
                tpms_protocol_bmw_analyze();
                if (callback) callback(this);
                packet_ready = false;
            }

            // Reset state machine
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            decode_data2 = 0;
            return;
        }

        bool bitstate;
        bool data_ok = FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate);

        if (data_ok) {
            // Shift bit into 128-bit sliding window
            decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;

            // 1. Check for 64-bit Audi Pressure Alert packet
            if (decode_count_bit >= 64) {
                uint8_t b[8], b_inv[8];
                bool all_0 = true, all_1 = true;

                for (int i = 0; i < 8; i++) {
                    b[i] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                    b_inv[i] = ~b[i];
                    if (b[i] != 0x00) all_0 = false;
                    if (b[i] != 0xFF) all_1 = false;
                }

                if (!all_0 && !all_1) {
                    // Check CRC-8 (Poly 0x2F, Init 0xAA) against first 7 bytes
                    if (FProtoGeneral::subghz_protocol_blocks_crc8(b, 7, 0x2F, 0xAA) == b[7]) {
                        saved_data = decode_data;
                        saved_type = MODEL_AUDI;
                        packet_ready = true;
                    } else if (FProtoGeneral::subghz_protocol_blocks_crc8(b_inv, 7, 0x2F, 0xAA) == b_inv[7]) {
                        saved_data = ~decode_data;
                        saved_type = MODEL_AUDI;
                        packet_ready = true;
                    }
                }
            }

            // 2. Check for 88-bit BMW Gen4/Gen5 packet
            if (decode_count_bit >= 88) {
                uint8_t b[11], b_inv[11];
                bool all_0 = true, all_1 = true;

                b[0] = (decode_data2 >> 16) & 0xFF;
                b[1] = (decode_data2 >> 8) & 0xFF;
                b[2] = (decode_data2) & 0xFF;
                b[3] = (decode_data >> 56) & 0xFF;
                b[4] = (decode_data >> 48) & 0xFF;
                b[5] = (decode_data >> 40) & 0xFF;
                b[6] = (decode_data >> 32) & 0xFF;
                b[7] = (decode_data >> 24) & 0xFF;
                b[8] = (decode_data >> 16) & 0xFF;
                b[9] = (decode_data >> 8) & 0xFF;
                b[10] = (decode_data) & 0xFF;

                for (int i = 0; i < 11; i++) {
                    b_inv[i] = ~b[i];
                    if (b[i] != 0x00) all_0 = false;
                    if (b[i] != 0xFF) all_1 = false;
                }

                if (!all_0 && !all_1) {
                    // Check CRC-8 (Poly 0x2F, Init 0xAA) against first 10 bytes
                    if (FProtoGeneral::subghz_protocol_blocks_crc8(b, 10, 0x2F, 0xAA) == b[10]) {
                        saved_data = decode_data;
                        saved_data2 = decode_data2;
                        saved_type = MODEL_BMW;  // Overwrites Audi false positive safely
                        packet_ready = true;
                    } else if (FProtoGeneral::subghz_protocol_blocks_crc8(b_inv, 10, 0x2F, 0xAA) == b_inv[10]) {
                        saved_data = ~decode_data;
                        saved_data2 = ~decode_data2;
                        saved_type = MODEL_BMW;
                        packet_ready = true;
                    }
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;

    // Hold the state dynamically until sequence drops off
    uint64_t saved_data = 0;
    uint64_t saved_data2 = 0;
    int saved_type = 0;
    bool packet_ready = false;
};

#endif