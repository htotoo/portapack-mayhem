#ifndef __FPROTO_TPMS_FORD_H__
#define __FPROTO_TPMS_FORD_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSFord : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSFord() {
        sensorType = FPT_Ford;  // Ensure this enum exists in your global definitions
        // rtl433 short/long width is 52us.
        // We set our half-bit period (te_short) to 52us and full-bit (te_long) to 104us
        te_short = 52;
        te_long = 104;
        te_delta = 25;  // Margin of error for FSK jitter
        min_count_bit_for_found = 64;
    }

    void tpms_protocol_ford_analyze(uint8_t* b) {
        // I = ID (Bytes 0 to 3)
        id = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];

        // Battery status isn't explicitly defined in this format
        battery = 0xFF;

        // P = Pressure (Byte 4 and 9th bit in Byte 6)
        int psibits = (((b[6] & 0x20) << 3) | b[4]);
        float pressure_psi = psibits * 0.25f;
        // Portapack/Flipper Zero typically expects TPMS pressure in Bar
        pressure = pressure_psi * 0.068947f;

        // T = Temperature (Byte 5)
        // Check 0x80 flag: if 0, then temperature is valid and has an offset of -56
        if ((b[5] & 0x80) == 0) {
            temperature = (b[5] & 0x7f) - 56;
        }
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        // Evaluate the pulse timing
        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Invalid duration for Manchester FSK.
            // Reset the state machine to wait for the next sync opportunity.
            FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
            decode_count_bit = 0;
            decode_data = 0;
            return;
        }

        bool bitstate;
        // Feed the valid pulse into the Manchester state machine
        bool data_ok = FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate);

        if (data_ok) {
            // Shift new bit into our 64-bit sliding window
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;

            // Once we have a full 64-bit frame, test the checksum on every new bit
            if (decode_count_bit >= min_count_bit_for_found) {
                uint8_t b[8];
                for (int i = 0; i < 8; i++) {
                    b[i] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                }

                uint8_t sum = 0;
                for (int i = 0; i < 7; i++) {
                    sum += b[i];
                }

                bool valid = false;

                // Eliminate all 0x00 and 0xFF dead-air noise triggers
                if (decode_data != 0 && decode_data != 0xFFFFFFFFFFFFFFFFULL) {
                    if (sum == b[7]) {
                        valid = true;
                    } else {
                        // The rtl_433 parser uses bitbuffer_invert. Test if the bit polarity is flipped.
                        uint8_t sum_inv = 0;
                        for (int i = 0; i < 7; i++) {
                            sum_inv += (uint8_t)(~b[i]);
                        }
                        if (sum_inv == (uint8_t)(~b[7])) {
                            valid = true;
                            decode_data = ~decode_data;  // Correct the data alignment

                            // Re-extract the corrected, inverted bytes for the analyze function
                            for (int i = 0; i < 8; i++) {
                                b[i] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                            }
                        }
                    }
                }

                if (valid) {
                    data_count_bit = 64;
                    // Extract payload into UI parameters
                    tpms_protocol_ford_analyze(b);
                    if (callback) callback(this);
                    // Reset to avoid repeated callbacks for the same packet
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