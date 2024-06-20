/*
 * Copyright (C) 2024 HTotoo
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "proc_wefaxrx.hpp"
#include "sine_table_int8.hpp"
#include "portapack_shared_memory.hpp"

#include "audio_dma.hpp"
#include "math.h"
#include "event_m4.hpp"
#include "fxpt_atan2.hpp"

#include <cstdint>
#include <cstddef>

#define M_PI 3.14159265358979323846

// updates the per pixel timers
void WeFaxRx::update_params() {
    switch (ioc_mode) {
        case 1:
            freq_start_tone = 675;
            break;
        default:
        case 0:
            freq_start_tone = 300;
            break;
    }
    // 840 px / line with line start
    time_per_pixel = 60000000 / lpm * 840;                            // micros (595,2380952 at 120 lpm)
    pxRem = (double)channel_filter_output_fs / ((double)lpm * 14.0);  // 840/60  = 228.57 sample / px
    samples_per_pixel = pxRem;
    pxRem -= (double)samples_per_pixel;
    pxRoll = pxRem;
}

double WeFaxRx::calculatePhaseAngle(int16_t i, int16_t q) {
    // return std::atan2(static_cast<double>(q), static_cast<double>(i));
    double ang = fxpt_atan2(q, i);
    return ang / 32768.0 * M_PI;
}

double WeFaxRx::calculateFrequencyDeviation(complex16_t& iq, complex16_t& iqlast) {
    // Calculate phase difference between successive samples
    double phaseDiff = calculatePhaseAngle(iq.imag(), iq.real()) - calculatePhaseAngle(iqlast.imag(), iqlast.real());
    // Ensure phase difference is within -pi to pi range
    if (phaseDiff > M_PI) {
        phaseDiff -= 2.0 * M_PI;
    } else if (phaseDiff < -M_PI) {
        phaseDiff += 2.0 * M_PI;
    }
    // Calculate frequency deviation
    return (phaseDiff / (2.0 * M_PI)) * channel_filter_output_fs;  // (sample rate)
}

void WeFaxRx::execute(const buffer_c8_t& buffer) {
    // This is called at 3072000 / 2048 = 1500Hz
    if (!configured) return;

    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);              // /8 = 256
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);         // /8 = 32
    const auto decim_2_out = decim_2.execute(decim_1_out, dst_buffer);         // /4 = 8
    const auto channel_out = channel_filter.execute(decim_2_out, dst_buffer);  // /1 = 8

    feed_channel_stats(channel_out);
    // auto audio = demod.execute(channel_out, audio_buffer);
    // audio_output.write(audio);

    // todo process
    for (size_t c = 0; c < channel_out.count; c++) {
        cnt++;
        double freqq = calculateFrequencyDeviation(channel_out.p[c], iqlast);

        if (status_message.freqmin > freqq) status_message.freqmin = freqq;
        if (status_message.freqmax < freqq) status_message.freqmax = freqq;
        if (freqq > 4000 || freqq < 1000) continue;
        status_message.freqavg += (freqq - status_message.freqavg) / cnt;
        iqlast = channel_out.p[c];
        if (cnt >= (samples_per_pixel + (uint32_t)pxRoll)) {  // got a pixel
            cnt = 0;
            if (pxRoll >= 1) pxRoll -= 1;
            pxRoll += pxRem;
            status_message.freq = freqq;
            image_message.cnt++;  // saves the pixel
            if (image_message.cnt < 380) {
                // 2300 = white
                // 1500 = black
                if (status_message.freqavg >= 3400)
                    image_message.image[image_message.cnt] = 255;
                else if (status_message.freqavg <= 1050)
                    image_message.image[image_message.cnt] = 0;
                else {
                    image_message.image[image_message.cnt] = 256 - (3400 - status_message.freqavg) / 3.15;
                }
            }
            if (image_message.cnt >= 840) {
                shared_memory.application_queue.push(image_message);
                image_message.cnt = 0;
                shared_memory.application_queue.push(status_message);
                status_message.freqmin = INT32_MAX;
                status_message.freqmax = INT32_MIN;
            }
            status_message.freqavg = 0;
        }
    }
}

void WeFaxRx::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::WeFaxRxConfigure:
        default:
            configure(*reinterpret_cast<const WeFaxRxConfigureMessage*>(message));
            break;
    }
}

void WeFaxRx::configure(const WeFaxRxConfigureMessage& message) {
    decim_0_input_fs = baseband_fs;
    decim_0_output_fs = decim_0_input_fs / decim_0.decimation_factor;
    decim_1_input_fs = decim_0_output_fs;
    decim_1_output_fs = decim_1_input_fs / decim_1.decimation_factor;
    decim_2_input_fs = decim_1_output_fs;
    decim_2_output_fs = decim_2_input_fs / 4;
    channel_filter_input_fs = decim_2_output_fs;
    channel_filter_output_fs = channel_filter_input_fs / 1;  // 12000ul
    update_params();
    lpm = message.lpm;
    ioc_mode = message.ioc;

    decim_0.configure(taps_6k0_decim_0.taps);
    decim_1.configure(taps_6k0_decim_1.taps);
    decim_2.configure(taps_6k0_decim_2.taps, 4);
    channel_filter.configure(taps_2k8_usb_channel.taps, 1);
    demod.configure(channel_filter_output_fs, 2800);
    audio_output.configure(audio_24k_hpf_300hz_config, audio_24k_deemph_300_6_config, 0);
    configured = true;
}

int main() {
    audio::dma::init_audio_out();
    EventDispatcher event_dispatcher{std::make_unique<WeFaxRx>()};
    event_dispatcher.run();
    return 0;
}
