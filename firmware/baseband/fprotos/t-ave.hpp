#ifndef __FPROTO_TPMS_AVE_H__
#define __FPROTO_TPMS_AVE_H__

#include "subtpmsbase.hpp"

class FProtoSubTPMSAVE : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSAVE() {
        sensorType = FPT_AVE;  // Ensure this enum exists in your global definitions

        // AVE TPMS uses FSK Differential Manchester
        // rtl_433 defines short/long as 100us raw ticks
        te_short = 100;
        te_long = 200;
        te_delta = 40;
        min_count_bit_for_found = 64;  // 8 bytes required
    }

    void tpms_protocol_ave_analyze(uint8_t* b) {
        // I = ID (Bytes 0 to 3)
        id = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];

        int pressure_raw = b[4];
        int temperature_raw = b[5];
        int mode = (b[6] >> 6) & 0x03;
        int battery_raw = (b[6] >> 3) & 0x07;

        // Battery status: 7 is low, 6 is not full, others are full
        if (battery_raw == 7) {
            battery = 25;
        } else if (battery_raw == 6) {
            battery = 75;
        } else {
            battery = 100;
        }

        // T = Temperature (Byte 5, offset by 50)
        temperature = temperature_raw - 50;

        // P = Pressure (Conversion depends on the 2-bit mode flag)
        float ratio = 2.352f;
        float offset = 0.0f;

        switch (mode) {
            case 0:
                ratio = 2.352f;
                offset = 47.0f;
                break;
            case 1:
                ratio = 2.352f;
                offset = 0.0f;
                break;
            case 2:
                ratio = 5.491f;
                offset = 18.2f;
                break;
            case 3:
                ratio = 5.491f;
                offset = 0.0f;
                break;
        }

        float pressure_kpa = ((float)pressure_raw - offset) * ratio;

        // Portapack expects TPMS pressure in Bar (1 Bar = 100 kPa)
        pressure = pressure_kpa * 0.01f;
    }

    void feed(bool level, uint32_t duration) {
        (void)level;  // Unused
        bool bitstate;
        bool data_ok = false;

        // Evaluate pulse timing for Differential Manchester:
        // Long pulse = '1', Two short pulses = '0'
        if (DURATION_DIFF(duration, te_short) < te_delta) {
            if (prev_short) {
                bitstate = 0;  // We received the second short -> emit '0'
                data_ok = true;
                prev_short = false;
            } else {
                prev_short = true;  // Wait for the second short pulse
            }
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            if (prev_short) {
                // Error: Expected a second short pulse, but got a long pulse.
                // This violates encoding rules. Reset sequence.
                decode_count_bit = 0;
                decode_data = 0;
                prev_short = false;
                return;
            }
            bitstate = 1;  // One long pulse -> emit '1'
            data_ok = true;
        } else {
            // Invalid duration -> Gap in transmission or noise resets the sequence
            decode_count_bit = 0;
            decode_data = 0;
            prev_short = false;
            return;
        }

        // If a bit was successfully decoded
        if (data_ok) {
            // Shift new bit into our 64-bit sliding window
            decode_data = (decode_data << 1) | bitstate;
            decode_count_bit++;

            // Wait until we have a full 64-bit frame to evaluate the CRC
            if (decode_count_bit >= min_count_bit_for_found) {
                uint8_t b[8];
                bool all_zeros = true;
                bool all_ones = true;

                // Extract window bytes
                for (int i = 0; i < 8; i++) {
                    b[i] = (decode_data >> ((7 - i) * 8)) & 0xFF;
                    if (b[i] != 0x00) all_zeros = false;
                    if (b[i] != 0xFF) all_ones = false;
                }

                // Eliminate pure dead-air noise triggers
                if (!all_zeros && !all_ones) {
                    // Standard CRC-8 (Poly 0x31, Init 0xFF).
                    // Running this over all 8 bytes (including the CRC byte itself) safely evaluates to 0 on success.
                    uint8_t crc = FProtoGeneral::subghz_protocol_blocks_crc8(b, 8, 0x31, 0xFF);

                    if (crc == 0x00) {
                        data_count_bit = 64;

                        tpms_protocol_ave_analyze(b);

                        if (callback) callback(this);

                        // Reset to avoid repeated callbacks for the same packet within the window
                        decode_count_bit = 0;
                        decode_data = 0;
                        prev_short = false;
                    }
                }
            }
        }
    }

   protected:
    // Replaces standard Manchester state machine. Tracks Differential short-pulse pairs.
    bool prev_short = false;
};

#endif