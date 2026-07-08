#include "proc_rds_rx.hpp"
#include "portapack_shared_memory.hpp"
#include "sine_table_int8.hpp"
#include "dsp_fir_taps.hpp"
#include "event_m4.hpp"
#include <cmath>

RDSProcessor::RDSProcessor() {
    decim_0.configure(taps_200k_wfm_decim_0.taps);
    demod_fm.configure(mpx_fs, 75000);
    decim_1.configure(taps_11k0_decim_1.taps);

    nco_inc = (57000ULL * 4294967296ULL) / mpx_fs;
    baseband_thread.start();
}

void RDSProcessor::execute(const buffer_c8_t& buffer) {
    const auto decim_0_out = decim_0.execute(buffer, dst_buffer_0);
    const auto mpx_out = demod_fm.execute(decim_0_out, mpx_buffer);
    feed_channel_stats(decim_0_out);

    for (size_t i = 0; i < mpx_out.count; i++) {
        float sample = mpx_out.p[i];

        uint8_t phase_idx = (nco_phase >> 24) & 0xFF;
        uint8_t cos_idx = (phase_idx + 64) & 0xFF;

        mixed[i] = {
            (int16_t)(sample * 32000.0f * sine_table_i8[cos_idx] / 128.0f),
            (int16_t)(sample * 32000.0f * sine_table_i8[phase_idx] / 128.0f)};
        nco_phase += nco_inc;
    }

    const buffer_c16_t mixed_buf_view{mixed.data(), mpx_out.count};
    const auto rds_out = decim_1.execute(mixed_buf_view, dst_buffer_1);

    for (size_t i = 0; i < rds_out.count; i++) {
        float i_sample = rds_out.p[i].real() / 2048.0f;
        float q_sample = rds_out.p[i].imag() / 2048.0f;

        float alpha = 0.2f;
        i_f1 += alpha * (i_sample - i_f1);
        q_f1 += alpha * (q_sample - q_f1);
        i_f2 += alpha * (i_f1 - i_f2);
        q_f2 += alpha * (q_f1 - q_f2);
        i_f3 += alpha * (i_f2 - i_f3);
        q_f3 += alpha * (q_f2 - q_f3);

        // FAST MATH: Kicserélve a CPU-gyilkos std::cos és std::sin hívásokat LUT-ra!
        int32_t phase_idx = (int32_t)(costas_phase * (128.0f / M_PI)) & 0xFF;
        float cos_p = sine_table_i8[(phase_idx + 64) & 0xFF] / 128.0f;
        float sin_p = sine_table_i8[phase_idx] / 128.0f;

        float i_rot = i_f3 * cos_p + q_f3 * sin_p;
        float q_rot = -i_f3 * sin_p + q_f3 * cos_p;

        float phase_err = (i_rot > 0.0f ? 1.0f : -1.0f) * q_rot;

        costas_freq += costas_beta * phase_err;
        costas_phase += (costas_alpha * phase_err) + costas_freq;

        if (costas_phase >= M_PI)
            costas_phase -= 2.0f * M_PI;
        else if (costas_phase < -M_PI)
            costas_phase += 2.0f * M_PI;

        clock_recovery(i_rot);
    }
}

void RDSProcessor::consume_symbol(const float raw_symbol) {
    uint8_t sym = (raw_symbol > 0.0f) ? 1 : 0;

    if (biphase_clock) {
        // Első fél: Dekódoljuk a bitet a MÁSIK bit második feléhez viszonyítva
        uint8_t bit = (sym != prev_sym) ? 0 : 1;
        process_bit(bit);

        first_half_sym = sym;  // Eltesszük az összehasonlításhoz
    } else {
        // Második fél: Ennek KÜLÖNBÖZNIE kell az első féltől!
        if (sym == first_half_sym) {
            // HIBA! Félrecsúsztunk a bithatárokon!
            // Automatikus fázis-igazítás, ezt a szimbólumot már egy új bit elejének tekintjük!
            biphase_clock = true;
            prev_sym = sym;
            return;
        }
        prev_sym = sym;  // Minden rendben, eltesszük a következő bithez
    }
    biphase_clock = !biphase_clock;
}

void RDSProcessor::process_bit(uint8_t bit) {
    bit_history = (bit_history << 1) | (bit & 0x01);
    bits_counted++;
    total_bits++;

    if (total_bits % 500 == 0) {
        uint16_t current_syndrome = calc_syndrome(bit_history & 0x03FFFFFF);
        uint32_t debug_val2 = (uint32_t)current_syndrome | ((uint32_t)sync_state << 16);
        RDSGroupMessage dbg_msg{0, 0, 0, 0, false, true, total_bits, debug_val2};
        shared_memory.application_queue.push(dbg_msg);
    }

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
                // Részleges csomagok felküldése
                if (sync_state == SyncState::EXPECT_C) {
                    RDSGroupMessage dbg_msg{block_a, block_b, 0, 0, false, true, 777, 0};
                    shared_memory.application_queue.push(dbg_msg);
                } else if (sync_state == SyncState::EXPECT_D) {
                    RDSGroupMessage dbg_msg{block_a, block_b, block_c, 0, false, true, 888, 0};
                    shared_memory.application_queue.push(dbg_msg);
                }
                sync_state = SyncState::UNSYNCED;
            }
            bits_counted = 0;
        }
    }
}

// Az igazi, matematikai szindróma-számoló, amivel az Offset szavak 100%-ban működnek
uint16_t RDSProcessor::calc_syndrome(uint32_t vec) {
    uint32_t reg = 0;
    for (int i = 25; i >= 0; i--) {
        reg = (reg << 1) | ((vec >> i) & 1);
        if (reg & 0x400) reg ^= 0x5B9;
    }
    return reg & 0x3FF;
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<RDSProcessor>()};
    event_dispatcher.run();
    return 0;
}