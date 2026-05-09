#ifndef __FPROTO_TPMS_HYUNDAI_VDO_H__
#define __FPROTO_TPMS_HYUNDAI_VDO_H__

#include "subtpmsbase.hpp"

typedef enum {
    HyundaiDecoderStepSearch = 0,
    HyundaiDecoderStepPayload
} HyundaiDecoderStep;

class FProtoSubTPMSHyundaiVDO : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSHyundaiVDO() {
        sensorType = FPT_HyundaiVDO;  // Ensure this enum exists in your global definitions

        te_short = 52;
        te_long = 104;
        te_delta = 25;
        min_count_bit_for_found = 80;  // Exactly 10 bytes expected
    }

    void tpms_protocol_hyundai_vdo_analyze(uint8_t* b) {
        // I = ID (Bytes 1 to 4)
        id = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];

        // Battery status is unmapped in this format, leaving as 0xFF
        battery = 0xFF;

        // P = Pressure (Byte 6)
        // b[6] * 1.375 = kPa. Portapack expects Bar (1 Bar = 100 kPa)
        pressure = ((float)b[6] * 1.375f) * 0.01f;

        // T = Temperature (Byte 7, offset by 50)
        temperature = (float)b[7] - 50.0f;
    }

    bool hyundai_sanity_check(uint8_t* b) {
        // 1. Validate ID is not empty or fully saturated
        if (b[1] == 0x00 && b[2] == 0x00 && b[3] == 0x00 && b[4] == 0x00) return false;
        if (b[1] == 0xFF && b[2] == 0xFF && b[3] == 0xFF && b[4] == 0xFF) return false;

        // 2. Validate known transmission states.
        // According to rtl_433, byte 0 is virtually always 0x20, 0x21, 0x22, or 0x23
        if (b[0] != 0x20 && b[0] != 0x21 && b[0] != 0x22 && b[0] != 0x23) return false;

        return true;
    }

    void reset_decoder() {
        FProtoGeneral::manchester_advance(manchester_saved_state, ManchesterEventReset, &manchester_saved_state, NULL);
        parser_step = HyundaiDecoderStepSearch;
        sync_reg = 0;
        decode_count_bit = 0;
        decode_data = 0;
        decode_data2 = 0;
        phase_inverted = false;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;

        if (DURATION_DIFF(duration, te_short) < te_delta) {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        } else if (DURATION_DIFF(duration, te_long) < te_delta) {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        } else {
            // Gap/Noise -> Hard reset the machine
            reset_decoder();
            return;
        }

        bool bitstate;
        bool data_ok = FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bitstate);

        if (data_ok) {
            if (parser_step == HyundaiDecoderStepSearch) {
                // Shift bits into a 32-bit preamble tracker
                sync_reg = (sync_reg << 1) | bitstate;

                // Hyundai VDO preamble is 0x55555556 (normal) or 0xAAAAAAA9 (inverted)
                if (sync_reg == 0x55555556 || sync_reg == 0xAAAAAAA9) {
                    parser_step = HyundaiDecoderStepPayload;
                    decode_count_bit = 0;
                    decode_data = 0;
                    decode_data2 = 0;
                    phase_inverted = (sync_reg == 0xAAAAAAA9);
                }
            } else if (parser_step == HyundaiDecoderStepPayload) {
                // Collect exactly 80 bits into our cascaded window
                decode_data2 = (decode_data2 << 1) | ((decode_data >> 63) & 1);
                decode_data = (decode_data << 1) | bitstate;
                decode_count_bit++;

                // Strictly evaluate only at the 80th bit
                if (decode_count_bit == 80) {
                    uint8_t b[10];
                    b[0] = (decode_data2 >> 8) & 0xFF;
                    b[1] = (decode_data2) & 0xFF;
                    b[2] = (decode_data >> 56) & 0xFF;
                    b[3] = (decode_data >> 48) & 0xFF;
                    b[4] = (decode_data >> 40) & 0xFF;
                    b[5] = (decode_data >> 32) & 0xFF;
                    b[6] = (decode_data >> 24) & 0xFF;
                    b[7] = (decode_data >> 16) & 0xFF;
                    b[8] = (decode_data >> 8) & 0xFF;
                    b[9] = (decode_data) & 0xFF;

                    // Apply FSK phase correction if the preamble was inverted
                    if (phase_inverted) {
                        for (int i = 0; i < 10; i++) b[i] = ~b[i];
                    }

                    // Check strict Hyundai Sanity Rules AND the mathematical CRC-8
                    if (hyundai_sanity_check(b) && FProtoGeneral::subghz_protocol_blocks_crc8(b, 9, 0x07, 0xAA) == b[9]) {
                        data_count_bit = 80;
                        tpms_protocol_hyundai_vdo_analyze(b);
                        if (callback) callback(this);
                    }

                    // Regardless of success/fail, drop back to searching for a new preamble
                    reset_decoder();
                }
            }
        }
    }

   protected:
    ManchesterState manchester_saved_state = ManchesterStateMid1;
    HyundaiDecoderStep parser_step = HyundaiDecoderStepSearch;
    uint32_t sync_reg = 0;
    bool phase_inverted = false;
};

#endif