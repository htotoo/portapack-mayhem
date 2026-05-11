/*
 * Copyright (C) 2026 HTotoo
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

#include "proc_subtpms.hpp"
#include "portapack_shared_memory.hpp"
#include "event_m4.hpp"

void SubTPMSProcessor::execute(const buffer_c8_t& buffer) {
    if (!configured) return;

    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);
    feed_channel_stats(decim_1_out);

    for (size_t i = 0; i < decim_1_out.count; i++) {
        int16_t re = decim_1_out.p[i].real();
        int16_t im = decim_1_out.p[i].imag();

        // AM (OOK) DEMODULATION
        if (modulation == 0) {
            uint32_t mag = ((uint32_t)re * (uint32_t)re) + ((uint32_t)im * (uint32_t)im);
            uint32_t am_mag = (mag >> 10);

            threshold = (low_estimate + high_estimate) / 2;
            int32_t const hysteresis = threshold / 8;
            int32_t const ook_low_delta = am_mag - low_estimate;
            bool meashl = currentHiLow;

            if (sig_state == STATE_IDLE) {
                if (am_mag > (threshold + hysteresis)) {
                    meashl = true;
                    sig_state = STATE_PULSE;
                    numg = 0;
                } else {
                    meashl = false;
                    low_estimate += ook_low_delta / OOK_EST_LOW_RATIO;
                    low_estimate += ((ook_low_delta > 0) ? 1 : -1);
                    high_estimate = 1.35 * low_estimate;
                    high_estimate = std::max(high_estimate, min_high_level);
                    high_estimate = std::min(high_estimate, (uint32_t)OOK_MAX_HIGH_LEVEL);
                }
            } else if (sig_state == STATE_PULSE) {
                ++numg;
                if (numg > 100) numg = 100;
                if (am_mag < (threshold - hysteresis)) {
                    if (numg < 3) {
                        sig_state = STATE_GAP;
                    } else {
                        numg = 0;
                        sig_state = STATE_GAP_START;
                    }
                    meashl = false;
                } else {
                    high_estimate += am_mag / OOK_EST_HIGH_RATIO - high_estimate / OOK_EST_HIGH_RATIO;
                    high_estimate = std::max(high_estimate, min_high_level);
                    high_estimate = std::min(high_estimate, (uint32_t)OOK_MAX_HIGH_LEVEL);
                    meashl = true;
                }
            } else if (sig_state == STATE_GAP_START) {
                ++numg;
                if (am_mag > (threshold + hysteresis)) {
                    sig_state = STATE_PULSE;
                    meashl = true;
                } else if (numg >= 3) {
                    sig_state = STATE_GAP;
                    meashl = false;
                }
            } else if (sig_state == STATE_GAP) {
                ++numg;
                if (am_mag > (threshold + hysteresis)) {
                    numg = 0;
                    sig_state = STATE_PULSE;
                    meashl = true;
                } else {
                    meashl = false;
                }
            }

            if (meashl == currentHiLow && currentDuration < 30'000'000) {
                currentDuration += nsPerDecSamp;
            } else {
                if (currentDuration >= 30'000'000) sig_state = STATE_IDLE;
                if (protoList) protoList->feed(currentHiLow, currentDuration / 1000);
                currentDuration = nsPerDecSamp;
                currentHiLow = meashl;
            }
        }

        // FM (FSK) DEMODULATION
        else if (modulation == 1) {
            int16_t re_s = re >> 2;
            int16_t im_s = im >> 2;

            int32_t discrim = ((int32_t)im_s * fm_state.last_re_s) - ((int32_t)re_s * fm_state.last_im_s);
            fm_state.last_re_s = re_s;
            fm_state.last_im_s = im_s;

            fm_state.smoothed_discrim += (discrim - fm_state.smoothed_discrim) >> 2;
            fm_state.dc_offset += (fm_state.smoothed_discrim - fm_state.dc_offset) >> 11;

            int32_t deviation = std::abs(fm_state.smoothed_discrim - fm_state.dc_offset);
            fm_state.deviation_avg += (deviation - fm_state.deviation_avg) >> 6;

            int32_t hysteresis = fm_state.deviation_avg >> 2;

            bool new_level = currentHiLow;
            if (fm_state.smoothed_discrim > fm_state.dc_offset + hysteresis) {
                new_level = true;
            } else if (fm_state.smoothed_discrim < fm_state.dc_offset - hysteresis) {
                new_level = false;
            }

            if (new_level == currentHiLow && currentDuration < 30'000'000) {
                currentDuration += nsPerDecSamp;
            } else {
                if (protoList) protoList->feed(currentHiLow, currentDuration / 1000);
                currentDuration = nsPerDecSamp;
                currentHiLow = new_level;
            }
        }
    }
}

void SubTPMSProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::SubGhzFPRxConfigure)
        configure(*reinterpret_cast<const SubGhzFPRxConfigureMessage*>(message));
}

void SubTPMSProcessor::configure(const SubGhzFPRxConfigureMessage& message) {
    if (modulation != message.modulation) {
        if (protoList) {
            delete protoList;
        }
        protoList = new SubTPMSProtos();
    }
    modulation = message.modulation;
    baseband_fs = message.sampling_rate;
    baseband_thread.set_sampling_rate(baseband_fs);
    nsPerDecSamp = 1'000'000'000 / baseband_fs * 8;

    decim_0.configure(taps_80k_wfm_decim_0.taps);
    decim_1.configure(taps_80k_wfm_decim_1.taps);

    configured = true;
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<SubTPMSProcessor>()};
    event_dispatcher.run();
    return 0;
}