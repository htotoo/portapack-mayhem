#include "proc_wideband_catch.hpp"
#include "portapack_shared_memory.hpp"
#include "event_m4.hpp"
#include "audio_dma.hpp"

WidebandCatchProcessor::WidebandCatchProcessor() {
    init_nco_lut();
    fm_demod.configure(AUDIO_RATE, 5000);
    audio_output.configure(false);
}

void WidebandCatchProcessor::init_nco_lut() {
    for (size_t i = 0; i < 256; i++) {
        float angle = (2.0f * 3.1415926535f * static_cast<float>(i)) / 256.0f;
        nco_lut[i] = {
            static_cast<int16_t>(std::cos(angle) * 256.0f),
            static_cast<int16_t>(std::sin(angle) * 256.0f)};
    }
}

void WidebandCatchProcessor::set_nco_frequency(int32_t freq_hz) {
    float normalized = static_cast<float>(-freq_hz) / static_cast<float>(SAMPLE_RATE);
    int64_t step = static_cast<int64_t>(normalized * 4294967296.0f);
    phase_inc = static_cast<uint32_t>(step);
}

void WidebandCatchProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::WidebandCatchConfig) {
        auto cfg = reinterpret_cast<const WidebandCatchConfigMessage*>(message);
        mod_type = cfg->mod_type;
        threshold = cfg->threshold;
        min_bins = cfg->min_bins;
        max_bins = cfg->max_bins;
        hold_samples_max = (cfg->squelch_hold_ms * AUDIO_RATE) / 1000;
        ignore_mask = cfg->ignore_mask;

        state = State::SCANNING;
    }
}

void WidebandCatchProcessor::execute(const buffer_c8_t& buffer) {
    if (state == State::SCANNING) {
        audio_buf.fill(0);
        buffer_s16_t audio_out_buf{audio_buf.data(), 32, AUDIO_RATE};
        audio_output.write(audio_out_buf);
        process_scanning(buffer);
    } else {
        process_locked(buffer);
    }
}

void WidebandCatchProcessor::process_scanning(const buffer_c8_t& buffer) {
    for (size_t i = 0; i < FFT_SIZE; i++) {
        fft_input_tmp[i] = {
            static_cast<int16_t>(buffer.p[i].real()),
            static_cast<int16_t>(buffer.p[i].imag())};
    }

    buffer_c16_t src_buf{fft_input_tmp.data(), FFT_SIZE};
    fft_swap(src_buf, channel_spectrum);

    fft_c_preswapped(channel_spectrum, 0, 8);

    for (size_t i = 0; i < FFT_SIZE; i++) {
        if (ignore_mask[i >> 3] & (1 << (i & 7))) {
            fft_mag[i] = 0.0f;
        } else {
            const float r = channel_spectrum[i].real();
            const float im = channel_spectrum[i].imag();
            fft_mag[i] = std::sqrt(r * r + im * im);
        }
    }

    fft_mag[0] = 0.0f;
    fft_mag[1] = 0.0f;
    fft_mag[FFT_SIZE - 1] = 0.0f;

    float thresh_f = static_cast<float>(threshold) * 16.0f;

    float best_peak_val = 0.0f;
    int best_peak_bin = -1;
    uint8_t best_width = 0;

    uint8_t current_width = 0;
    float current_max_val = 0.0f;
    int current_max_bin = -1;

    for (size_t idx = 0; idx < FFT_SIZE; idx++) {
        size_t b = (idx + FFT_SIZE / 2) % FFT_SIZE;
        float mag = fft_mag[b];

        if (mag > thresh_f) {
            current_width++;
            if (mag > current_max_val) {
                current_max_val = mag;
                current_max_bin = static_cast<int>(b);
            }
        } else {
            if (current_width > 0) {
                if (current_width >= min_bins && current_width <= max_bins) {
                    if (current_max_val > best_peak_val) {
                        best_peak_val = current_max_val;
                        best_peak_bin = current_max_bin;
                        best_width = current_width;
                    }
                }
                current_width = 0;
                current_max_val = 0.0f;
            }
        }
    }

    if (best_peak_bin != -1) {
        size_t b_left = (best_peak_bin + FFT_SIZE - 1) % FFT_SIZE;
        size_t b_right = (best_peak_bin + 1) % FFT_SIZE;
        float y1 = fft_mag[b_left];
        float y2 = fft_mag[best_peak_bin];
        float y3 = fft_mag[b_right];

        float delta = 0.0f;
        float denom = (y1 - 2.0f * y2 + y3);
        if (std::abs(denom) > 0.001f) {
            delta = 0.5f * (y1 - y3) / denom;
        }

        int signed_bin = (best_peak_bin < static_cast<int>(FFT_SIZE / 2))
                             ? best_peak_bin
                             : (best_peak_bin - static_cast<int>(FFT_SIZE));

        float exact_bin = static_cast<float>(signed_bin) + delta;
        float bin_hz = static_cast<float>(SAMPLE_RATE) / static_cast<float>(FFT_SIZE);

        locked_freq_offset = static_cast<int32_t>(exact_bin * bin_hz);
        set_nco_frequency(locked_freq_offset);

        cic_i_integ1 = cic_i_integ2 = cic_q_integ1 = cic_q_integ2 = 0;
        cic_i_comb1 = cic_i_comb2 = cic_q_comb1 = cic_q_comb2 = 0;
        decim_counter = 0;
        hold_counter = hold_samples_max;
        state = State::LOCKED;

        send_status(true, locked_freq_offset, static_cast<uint16_t>(best_peak_val / 16.0f), best_width);
    }
}

void WidebandCatchProcessor::process_locked(const buffer_c8_t& buffer) {
    uint32_t power_acc = 0;
    size_t baseband_count = 0;

    for (size_t i = 0; i < buffer.count; i++) {
        int16_t s_i = buffer.p[i].real();
        int16_t s_q = buffer.p[i].imag();

        uint8_t lut_idx = (phase_acc >> 24) & 0xFF;
        phase_acc += phase_inc;

        int16_t lo_i = nco_lut[lut_idx].real();
        int16_t lo_q = nco_lut[lut_idx].imag();

        int32_t mix_i = (s_i * lo_i - s_q * lo_q) >> 8;
        int32_t mix_q = (s_i * lo_q + s_q * lo_i) >> 8;

        cic_i_integ1 += mix_i;
        cic_i_integ2 += cic_i_integ1;
        cic_q_integ1 += mix_q;
        cic_q_integ2 += cic_q_integ1;

        if (++decim_counter >= DECIMATION_FACTOR) {
            decim_counter = 0;

            int32_t d1_i = cic_i_integ2 - cic_i_comb1;
            cic_i_comb1 = cic_i_integ2;
            int32_t out_i = (d1_i - cic_i_comb2) >> 10;
            cic_i_comb2 = d1_i;

            int32_t d1_q = cic_q_integ2 - cic_q_comb1;
            cic_q_comb1 = cic_q_integ2;
            int32_t out_q = (d1_q - cic_q_comb2) >> 10;
            cic_q_comb2 = d1_q;

            uint32_t mag_sq = static_cast<uint32_t>(out_i * out_i + out_q * out_q);
            uint16_t mag = std::sqrt(mag_sq);
            power_acc += mag;

            baseband_buf[baseband_count++] = {
                static_cast<int16_t>(out_i),
                static_cast<int16_t>(out_q)};

            if (baseband_count == 32) {
                buffer_s16_t audio_out_buf{audio_buf.data(), 32, AUDIO_RATE};

                if (mod_type == 0) {
                    // NFM
                    buffer_c16_t c16_buf{baseband_buf.data(), 32, AUDIO_RATE};
                    fm_demod.execute(c16_buf, audio_out_buf);
                } else {
                    // AM
                    for (size_t k = 0; k < 32; k++) {
                        int32_t ii = baseband_buf[k].real();
                        int32_t qq = baseband_buf[k].imag();
                        int32_t env = static_cast<int32_t>(std::sqrt(ii * ii + qq * qq)) << 4;

                        am_dc_est += (env - am_dc_est) >> 6;
                        int32_t ac_audio = env - am_dc_est;

                        if (ac_audio > 32767)
                            ac_audio = 32767;
                        else if (ac_audio < -32768)
                            ac_audio = -32768;

                        audio_buf[k] = static_cast<int16_t>(ac_audio);
                    }
                }

                audio_output.write(audio_out_buf);
                baseband_count = 0;
            }
        }
    }

    // Squelch
    uint32_t decimated_samples = buffer.count / DECIMATION_FACTOR;
    uint16_t avg_level = (decimated_samples > 0) ? (power_acc / decimated_samples) : 0;

    if (avg_level >= threshold) {
        hold_counter = hold_samples_max;
    } else {
        if (hold_counter > decimated_samples) {
            hold_counter -= decimated_samples;
        } else {
            state = State::SCANNING;
            am_dc_est = 0;
            std::fill(audio_buf.begin(), audio_buf.end(), 0);
            audio_output.write({audio_buf.data(), 32, AUDIO_RATE});
            send_status(false, 0, 0, 0);
            return;
        }
    }

    if (++status_update_counter >= 15) {
        status_update_counter = 0;
        send_status(true, locked_freq_offset, avg_level, 0);
    }
}

void WidebandCatchProcessor::send_status(bool locked, int32_t offset, uint16_t level, uint8_t width) {
    WidebandCatchStatusMessage msg{locked, offset, level, width};
    shared_memory.application_queue.push(msg);
}

int main() {
    audio::dma::init_audio_out();
    EventDispatcher event_dispatcher{std::make_unique<WidebandCatchProcessor>()};
    event_dispatcher.run();
    return 0;
}