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

        // ==========================================
        // AM (OOK) DEMODULATION - ULTRALIGHT VERSION
        // ==========================================
        if (modulation == 0) {
            // Nyers magnitúdó számítás
            uint32_t am_mag = (((uint32_t)re * re) + ((uint32_t)im * im)) >> 10;

            // Osztás bit-eltolással (>> 1 = osztás 2-vel, >> 3 = osztás 8-cal)
            threshold = (low_estimate + high_estimate) >> 1;
            int32_t hysteresis = threshold >> 3;
            if (hysteresis < 20) hysteresis = 20;  // Squelch zajküszöb

            int32_t const ook_low_delta = am_mag - low_estimate;
            bool meashl = currentHiLow;  // Alapból feltételezzük, hogy a szint maradt

            // JUMP-TABLE (SWITCH) ALAPÚ ÁLLAPOTGÉP
            switch (sig_state) {
                case STATE_IDLE:
                    if (am_mag > (threshold + hysteresis)) {
                        meashl = true;
                        sig_state = STATE_PULSE;
                        numg = 0;
                    } else {
                        meashl = false;
                        low_estimate += ook_low_delta / OOK_EST_LOW_RATIO;
                        low_estimate += ((ook_low_delta > 0) ? 1 : -1);

                        // FLOAT GYILKOSSÁG: 1.35 * low_estimate helyett fixed-point matek!
                        high_estimate = (low_estimate * 345) >> 8;

                        high_estimate = (high_estimate > min_high_level) ? high_estimate : min_high_level;
                        high_estimate = (high_estimate < (uint32_t)OOK_MAX_HIGH_LEVEL) ? high_estimate : (uint32_t)OOK_MAX_HIGH_LEVEL;
                    }
                    break;

                case STATE_PULSE:
                    if (++numg > 100) numg = 100;
                    if (am_mag < (threshold - hysteresis)) {
                        if (numg < 3)
                            sig_state = STATE_GAP;
                        else {
                            numg = 0;
                            sig_state = STATE_GAP_START;
                        }
                        meashl = false;
                    } else {
                        high_estimate += (am_mag - high_estimate) / OOK_EST_HIGH_RATIO;
                        high_estimate = (high_estimate > min_high_level) ? high_estimate : min_high_level;
                        high_estimate = (high_estimate < (uint32_t)OOK_MAX_HIGH_LEVEL) ? high_estimate : (uint32_t)OOK_MAX_HIGH_LEVEL;
                        meashl = true;
                    }
                    break;

                case STATE_GAP_START:
                    if (am_mag > (threshold + hysteresis)) {
                        sig_state = STATE_PULSE;
                        meashl = true;
                    } else if (++numg >= 3) {
                        sig_state = STATE_GAP;
                        meashl = false;
                    }
                    break;

                case STATE_GAP:
                    if (am_mag > (threshold + hysteresis)) {
                        numg = 0;
                        sig_state = STATE_PULSE;
                        meashl = true;
                    } else {
                        meashl = false;
                    }
                    break;
            }

            // --- AM ZAVAR-ELNYELŐ (GLITCH-SWALLOWER) ---
            if (meashl == currentHiLow) {
                // A jel egyezik a stabil fázissal
                currentDuration += nsPerDecSamp;
                if (glitchDuration > 0) {
                    // Tüskére elpazarolt idő visszaszolgáltatása
                    currentDuration += glitchDuration;
                    glitchDuration = 0;
                }
            } else {
                // A jel eltér a stabil fázistól (Tüske, vagy valódi élváltás kezdete)
                glitchDuration += nsPerDecSamp;
                if (glitchDuration >= 15'000) {
                    // Tüske túlélte a 15us-ot -> Hivatalos élváltás!
                    if (protoList) protoList->feed(currentHiLow, currentDuration / 1000);
                    currentHiLow = meashl;
                    currentDuration = glitchDuration;
                    glitchDuration = 0;
                }
            }

            if (currentDuration >= 30'000'000) {
                if (protoList) protoList->feed(currentHiLow, currentDuration / 1000);
                currentDuration = 0;
                glitchDuration = 0;
                sig_state = STATE_IDLE;
            }
        }

        // ==========================================
        // FM (FSK) DEMODULATION - ULTRALIGHT VERSION
        // ==========================================
        else if (modulation == 1) {
            int16_t re_s = re >> 2;
            int16_t im_s = im >> 2;

            // Hardveres 32-bites MAC (Multiply-Accumulate) kihasználása
            int32_t discrim = (im_s * fm_state.last_re_s) - (re_s * fm_state.last_im_s);
            fm_state.last_re_s = re_s;
            fm_state.last_im_s = im_s;

            fm_state.smoothed_discrim += (discrim - fm_state.smoothed_discrim) >> 2;
            fm_state.dc_offset += (fm_state.smoothed_discrim - fm_state.dc_offset) >> 11;

            // Branchless (ugrás nélküli) abszolút érték számítás
            int32_t temp = fm_state.smoothed_discrim - fm_state.dc_offset;
            int32_t mask = temp >> 31;
            int32_t deviation = (temp + mask) ^ mask;

            fm_state.deviation_avg += (deviation - fm_state.deviation_avg) >> 6;

            int32_t hysteresis = fm_state.deviation_avg >> 2;
            if (hysteresis < 1024) hysteresis = 1024;  // FM Squelch

            bool raw_level = currentHiLow;  // A holtsávban feltételezzük, hogy a jel marad
            if (fm_state.smoothed_discrim > fm_state.dc_offset + hysteresis) {
                raw_level = true;
            } else if (fm_state.smoothed_discrim < fm_state.dc_offset - hysteresis) {
                raw_level = false;
            }

            // --- FM ZAVAR-ELNYELŐ (GLITCH-SWALLOWER) ---
            if (raw_level == currentHiLow) {
                currentDuration += nsPerDecSamp;
                if (glitchDuration > 0) {
                    currentDuration += glitchDuration;
                    glitchDuration = 0;
                }
            } else {
                glitchDuration += nsPerDecSamp;
                if (glitchDuration >= 15'000) {
                    if (protoList) protoList->feed(currentHiLow, currentDuration / 1000);
                    currentHiLow = raw_level;
                    currentDuration = glitchDuration;
                    glitchDuration = 0;
                }
            }

            if (currentDuration >= 30'000'000) {
                if (protoList) protoList->feed(currentHiLow, currentDuration / 1000);
                currentDuration = 0;
                glitchDuration = 0;
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