#include "proc_rds_rx.hpp"
#include "portapack_shared_memory.hpp"
#include "sine_table_int8.hpp"
#include "dsp_fir_taps.hpp"
#include "event_m4.hpp"

RDSProcessor::RDSProcessor() {
    decim_0.configure(taps_200k_wfm_decim_0.taps);
    demod_fm.configure(mpx_fs, 75000);

    nco_inc = (57000ULL * 4294967296ULL) / mpx_fs;
    baseband_thread.start();
}

void RDSProcessor::execute(const buffer_c8_t& buffer) {
    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
    const auto mpx_out = demod_fm.execute(decim_0_out, mpx_buffer);
    feed_channel_stats(decim_0_out);

    for (size_t i = 0; i < mpx_out.count; i++) {
        int16_t sample = mpx_out.p[i];

        uint8_t phase_idx = (nco_phase >> 24) & 0xFF;
        uint8_t cos_idx = (phase_idx + 64) & 0xFF;

        float i_mixed = (sample * sine_table_i8[cos_idx]) / 128.0f;
        float q_mixed = (sample * sine_table_i8[phase_idx]) / 128.0f;

        // KASZKÁDOLT IIR SZŰRŐ: Brutálisan levágja a sztereó hangot!
        // 1. A szűrőt kicsit kinyitjuk (0.05f-ről 0.1f-re),
        // hogy a 2.4 kHz széles RDS jel garantáltan ne tompuljon el
        i_f1 += 0.1f * (i_mixed - i_f1);
        q_f1 += 0.1f * (q_mixed - q_f1);
        i_f2 += 0.1f * (i_f1 - i_f2);
        q_f2 += 0.1f * (q_f1 - q_f2);

        // 2. NORMALIZÁLÁS JAVÍTÁSA: 8000 helyett 1000-rel osztunk!
        // Így a jelszint felmegy a tökéletes ~0.9-es értékre, a hurok azonnal rázár.
        float i_norm = i_f2 / 1000.0f;
        float q_norm = q_f2 / 1000.0f;

        float phase_err = (i_norm > 0.0f ? 1.0f : -1.0f) * q_norm;

        costas_freq += costas_beta * phase_err;
        float phase_adj = (costas_alpha * phase_err) + costas_freq;

        nco_phase += nco_inc + (int32_t)(phase_adj * 683565275.0f);

        clock_recovery(i_norm);
    }
}

void RDSProcessor::consume_symbol(const float raw_symbol) {
    uint8_t current_symbol = (raw_symbol > 0.0f) ? 1 : 0;

    symbol_count++;
    if (symbol_count % 2375 == 0) {
        uint16_t current_syndrome = calc_syndrome(bit_history & 0x03FFFFFF);
        uint32_t debug_val2 = (uint32_t)current_syndrome | ((uint32_t)sync_state << 16);
        RDSGroupMessage dbg_msg{0, 0, 0, 0, false, true, symbol_count, debug_val2};
        shared_memory.application_queue.push(dbg_msg);
    }

    // A ZSENIÁLIS BIPHASE DEKÓDER: Csak egy flip-flop!
    biphase_flip = !biphase_flip;

    if (biphase_flip) {
        // Differenciális NRZI dekódolás minden 2. szimbólumon
        uint8_t decoded_bit = current_symbol ^ last_diff_bit;
        last_diff_bit = current_symbol;

        process_bit(decoded_bit);
    }
}

void RDSProcessor::process_bit(uint8_t bit) {
    bit_history = (bit_history << 1) | (bit & 0x01);
    bits_counted++;

    if (sync_state == SyncState::UNSYNCED) {
        if (calc_syndrome(bit_history & 0x03FFFFFF) == SYNDROME_A) {
            sync_state = SyncState::EXPECT_B;
            block_a = (bit_history >> 10) & 0xFFFF;
            bits_counted = 0;
        }
    } else {
        if (bits_counted == 26) {
            uint16_t syndrome = calc_syndrome(bit_history & 0x03FFFFFF);
            uint16_t data = (bit_history >> 10) & 0xFFFF;

            if (sync_state == SyncState::EXPECT_B && syndrome == SYNDROME_B) {
                block_b = data;
                sync_state = SyncState::EXPECT_C;
            } else if (sync_state == SyncState::EXPECT_C && (syndrome == SYNDROME_C || syndrome == SYNDROME_Cp)) {
                block_c = data;
                is_c_prime = (syndrome == SYNDROME_Cp);
                sync_state = SyncState::EXPECT_D;
            } else if (sync_state == SyncState::EXPECT_D && syndrome == SYNDROME_D) {
                block_d = data;

                RDSGroupMessage msg{block_a, block_b, block_c, block_d, is_c_prime, false, 0, 0};
                shared_memory.application_queue.push(msg);

                sync_state = SyncState::EXPECT_A;
            } else if (sync_state == SyncState::EXPECT_A && syndrome == SYNDROME_A) {
                block_a = data;
                sync_state = SyncState::EXPECT_B;
            } else {
                sync_state = SyncState::UNSYNCED;
            }
            bits_counted = 0;
        }
    }
}

uint16_t RDSProcessor::calc_syndrome(uint32_t vec) {
    uint32_t reg = 0;
    for (int i = 25; i >= 0; i--) {
        uint8_t reg_out = (reg >> 9) & 1;
        reg = ((reg << 1) & 0x3FF) | ((vec >> i) & 1);
        if (reg_out) reg ^= 0x1B9;
    }
    return reg;
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<RDSProcessor>()};
    event_dispatcher.run();
    return 0;
}