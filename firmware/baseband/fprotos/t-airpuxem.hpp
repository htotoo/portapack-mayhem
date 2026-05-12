#ifndef __FPROTO_TPMS_AIRPUXEM_H__
#define __FPROTO_TPMS_AIRPUXEM_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSAirpuxem : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSAirpuxem() {
        sensorType = FPT_Airpuxem;  // Ensure this enum exists in your global definitions

        // rtl_433 defines short/long width as 52us for this FSK PCM.
        te_short = 52;
        te_long = 104;
        te_delta = 25;
        min_count_bit_for_found = 84;  // 4-bit header + 64-bit payload + 8-bit CRC + 8-bit repeated CRC
    }

    void tpms_protocol_airpuxem_analyze(uint8_t* payload) {
        // I = ID (Bytes 0 to 3 of the payload)
        id = ((uint32_t)payload[0] << 24) | ((uint32_t)payload[1] << 16) | ((uint32_t)payload[2] << 8) | payload[3];

        // P = Pressure (payload[5] combined with specific bits from payload[4])
        // Formula extracts 10-bit value and subtracts 100 to get kPa
        int pressure_kpa = (payload[5] | (((payload[4] >> 7) & 1) << 8) | (((payload[4] >> 3) & 1) << 9)) - 100;

        // Portapack expects Bar (1 Bar = 100 kPa)
        pressure = (float)pressure_kpa * 0.01f;

        // T = Temperature (Byte 6, standard signed 8-bit integer)
        temperature = (int8_t)payload[6];

        // B = Battery level (Byte 7).
        // rtl_433 applies * 0.02V formatting, but Portapack base variables typically
        // store the raw integer byte for UI interpretation. We pass the raw byte here.
        battery = payload[7];
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        // Evaluate the pulse timing
        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Sequence break / gap resets the state machine
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

            // Wait until we have a full 84-bit frame to evaluate
            if (decode_count_bit >= min_count_bit_for_found) {
                if (decode_data == 0ULL || decode_data == ~0ULL) return;
                // The header sits exactly 80 bits back (bits 16-19 inside decode_data2)
                uint8_t header = (decode_data2 >> 16) & 0x0F;

                // Validate if it matches the '0x5' sync nibble (or '0xA' if FSK phase is inverted)
                if (header == 0x05 || header == 0x0A) {
                    // Temporary stack array to run mathematical CRC against (freed instantly)
                    uint8_t payload[8];

                    // Extract exactly 64 bits starting immediately after the 4-bit header
                    payload[0] = (decode_data2 >> 8) & 0xFF;
                    payload[1] = (decode_data2) & 0xFF;
                    payload[2] = (decode_data >> 56) & 0xFF;
                    payload[3] = (decode_data >> 48) & 0xFF;
                    payload[4] = (decode_data >> 40) & 0xFF;
                    payload[5] = (decode_data >> 32) & 0xFF;
                    payload[6] = (decode_data >> 24) & 0xFF;
                    payload[7] = (decode_data >> 16) & 0xFF;

                    // Extract the primary 8-bit CRC chunk
                    uint8_t crc1 = (decode_data >> 8) & 0xFF;

                    // If the header showed an inverted phase, invert our payload and CRC before testing
                    bool invert = (header == 0x0A);
                    if (invert) {
                        for (int i = 0; i < 8; i++) {
                            payload[i] = ~payload[i];
                        }
                        crc1 = ~crc1;
                    }

                    // Check CRC-8 (Poly 0x2F, Init 0xAA) against the 8-byte payload
                    uint8_t crc_calc = FProtoGeneral::subghz_protocol_blocks_crc8(payload, 8, 0x2F, 0xAA);

                    if (crc_calc == crc1) {
                        data_count_bit = 84;

                        // Pass validated, polarity-corrected payload to the analyzer
                        tpms_protocol_airpuxem_analyze(payload);
                        if (callback) callback(this);

                        // Reset to avoid duplicate triggers within the same broadcast burst
                        decode_count_bit = 0;
                        decode_data = 0;
                        decode_data2 = 0;
                        FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                    }
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif