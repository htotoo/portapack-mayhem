#ifndef __FPROTO_TPMS_FORD_H__
#define __FPROTO_TPMS_FORD_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSFord : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSFord() {
        sensorType = FPT_Ford;
        te_short = 52;
        te_long = 104;
        te_delta = 25;
        min_count_bit_for_found = 64;
    }

    void tpms_protocol_ford_analyze(uint8_t* b) {
        id = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
        battery = 0xFF;

        int psibits = (((b[6] & 0x20) << 3) | b[4]);
        pressure = (psibits * 0.25f) * 0.068947f;  // Convert PSI to Bar

        if ((b[5] & 0x80) == 0) {
            temperature = (b[5] & 0x7f) - 56;
        }
    }

    // Mathematically filter out false-positives from BMW/Abarth/Hyundai overlapping signals
    bool ford_payload_sanity_check(uint8_t* b) {
        // 1. Validate Checksum (Sum of bytes 0-6 must equal byte 7)
        uint8_t sum = 0;
        for (int i = 0; i < 7; i++) {
            sum += b[i];
        }
        if (sum != b[7]) return false;

        // 2. Validate Flags (Byte 6) against known Ford parameters
        uint8_t flags = b[6];

        // According to rtl_433, bits 7 (0x80) and 4 (0x10) are unused and must be 0
        if ((flags & 0x90) != 0x00) return false;

        // Bits 6, 3, and 2 define the transmission state.
        // Must exactly match: 0x08 (Learn), 0x04 (At Rest), or 0x44 (Moving)
        uint8_t state = flags & 0x4C;
        if (state != 0x08 && state != 0x04 && state != 0x44) return false;

        // 3. ID cannot be completely empty
        if (b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0) return false;

        return true;  // Packet is confirmed as a legitimate Ford transmission
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Gap/Noise: Reset sequence
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            return;
        }

        bool bitstate;
        bool data_ok = FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate);

        if (data_ok) {
            // Shift new bit into our 64-bit continuous sliding window
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;

            // Evaluate payload dynamically
            if (decode_count_bit >= min_count_bit_for_found) {
                uint8_t b[8];
                uint8_t b_inv[8];

                for (int i = 0; i < 8; i++) {
                    b[i] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                    b_inv[i] = ~b[i];  // Prepare inverted phase array
                }

                // If the packet perfectly passes the checksum AND the strict Ford sanity rules
                if (ford_payload_sanity_check(b)) {
                    data_count_bit = 64;
                    tpms_protocol_ford_analyze(b);
                    if (callback) callback(this);

                    // Reset to avoid duplicate UI triggers
                    decode_count_bit = 0;
                    decode_data = 0;
                    FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                }
                // Test if the FSK phase was inverted
                else if (ford_payload_sanity_check(b_inv)) {
                    data_count_bit = 64;
                    tpms_protocol_ford_analyze(b_inv);
                    if (callback) callback(this);

                    decode_count_bit = 0;
                    decode_data = 0;
                    FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
};

#endif