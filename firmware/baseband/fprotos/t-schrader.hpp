#pragma once
#include "subtpmsbase.hpp"
#include <cstring>

// https://github.com/merbanan/rtl_433/blob/master/src/devices/schraeder.c
// https://elib.dlr.de/81155/1/TPMS_for_Trafffic_Management_purposes.pdf
// https://github.com/furrtek/portapack-havoc/issues/349
// https://fccid.io/MRXGG4
// https://fccid.io/MRXGG4T

/**
 * Schrader 3013/3015 MRX-GG4

OEM
KIA Sportage CGA 11-SPT1504-RA
Mercedes-Benz A0009054100

* Frequency: 433.92MHz+-38KHz
* Modulation: ASK
* Working Temperature: -50°C to 125°C
* Tire monitoring range value: 0kPa-350kPa+-7kPa

Examples in normal environmental conditions:
3000878456094cd0
3000878456084ecb
3000878456074d01

Data layout:
 * | Byte 0    | Byte 1    | Byte 2    | Byte 3    | Byte 4    | Byte 5    | Byte 6    | Byte 7    |
 * | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- |
 * | SSSS SSSS | IIII IIII | IIII IIII | IIII IIII | IIII IIII | PPPP PPPP | TTTT TTTT | CCCC CCCC |
 *

- The preamble is 0b000
- S: always 0x30 in relearn state
- I: 32 bit ID
- P: 8 bit Pressure (multiplyed by 2.5 = PSI)
- T: 8 bit Temperature (deg. C offset by 50)
- C: 8 bit Checksum (CRC8, Poly 0x7, Init 0x0)
*/

#define PREAMBLE 0b000
#define PREAMBLE_BITS_LEN 3

typedef enum {
    SchraderGG4DecoderStepReset = 0,
    SchraderGG4DecoderStepCheckPreamble,
    SchraderGG4DecoderStepDecoderData,
    SchraderGG4DecoderStepSaveDuration,
    SchraderGG4DecoderStepCheckDuration,
} SchraderGG4DecoderStep;

class FProtoSubTPMSSchrader : public FProtoSubTPMSBase {
   public:
    FProtoSubTPMSSchrader() {
        sensorType = FPT_Schrader;
        te_short = 120;
        te_long = 240;
        te_delta = 55;
        min_count_bit_for_found = 64;
    }

    bool tpms_protocol_schrader_gg4_check_crc() {
        uint8_t msg[] = {
            uint8_t(decode_data >> 48),
            uint8_t(decode_data >> 40),
            uint8_t(decode_data >> 32),
            uint8_t(decode_data >> 24),
            uint8_t(decode_data >> 16),
            uint8_t(decode_data >> 8)};

        uint8_t crc = FProtoGeneral::subghz_protocol_blocks_crc8(msg, 6, 0x7, 0);
        return (crc == (decode_data & 0xFF));
    }

    ManchesterEvent level_and_duration_to_event(bool level, uint32_t duration) {
        bool is_long = false;

        if (DURATION_DIFF(duration, te_long) <
            te_delta) {
            is_long = true;
        } else if (
            DURATION_DIFF(duration, te_short) <
            te_delta) {
            is_long = false;
        } else {
            return ManchesterEventReset;
        }

        if (level)
            return is_long ? ManchesterEventLongHigh : ManchesterEventShortHigh;
        else
            return is_long ? ManchesterEventLongLow : ManchesterEventShortLow;
    }

    void tpms_protocol_schrader_gg4_analyze() {
        id = decode_data >> 24;
        battery = 0xFF;
        temperature = ((decode_data >> 8) & 0xFF) - 50;
        pressure = ((decode_data >> 16) & 0xFF) * 2.5 * 0.069;
    }

    void feed(bool level, uint32_t duration) {
        bool bit = false;
        bool have_bit = false;

        // low-level bit sequence decoding
        if (parser_step != SchraderGG4DecoderStepReset) {
            ManchesterEvent event = level_and_duration_to_event(level, duration);

            if (event == ManchesterEventReset) {
                if ((parser_step == SchraderGG4DecoderStepDecoderData) && decode_count_bit) {
                    // FURI_LOG_D(TAG, "%d-%ld", level, duration);
                }

                parser_step = SchraderGG4DecoderStepReset;
            } else {
                have_bit = FProtoGeneral::manchester_advance(manchester_saved_state, event, &manchester_saved_state, &bit);
                if (!have_bit) return;
                // Invert value, due to signal is Manchester II and decoder is Manchester I
                bit = !bit;
            }
        }

        switch (parser_step) {
            case SchraderGG4DecoderStepReset:
                // wait for start ~480us pulse
                if ((level) && (DURATION_DIFF(duration, te_long * 2) < te_delta)) {
                    parser_step = SchraderGG4DecoderStepCheckPreamble;
                    header_count = 0;
                    decode_data = 0;
                    decode_count_bit = 0;
                    // First will be short space, so set correct initial state for machine
                    // https://clearwater.com.au/images/rc5/rc5-state-machine.gif
                    manchester_saved_state = ManchesterStateStart1;
                }
                break;
            case SchraderGG4DecoderStepCheckPreamble:
                if (bit != 0) {
                    parser_step = SchraderGG4DecoderStepReset;
                    break;
                }

                header_count++;
                if (header_count == PREAMBLE_BITS_LEN)
                    parser_step = SchraderGG4DecoderStepDecoderData;
                break;

            case SchraderGG4DecoderStepDecoderData:
                subghz_protocol_blocks_add_bit(bit);
                if (decode_count_bit ==
                    min_count_bit_for_found) {
                    if (!tpms_protocol_schrader_gg4_check_crc()) {
                        // FURI_LOG_D(TAG, "CRC mismatch drop");
                    } else {
                        data_count_bit = decode_count_bit;
                        tpms_protocol_schrader_gg4_analyze();
                        if (callback)
                            callback(this);
                    }
                    parser_step = SchraderGG4DecoderStepReset;
                }
                break;
        }
    }

   private:
    uint8_t header_count = 0;
    ManchesterState manchester_saved_state = ManchesterStateStart1;
};
