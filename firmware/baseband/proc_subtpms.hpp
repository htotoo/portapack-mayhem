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

#ifndef __PROC_SUBTPMS_H__
#define __PROC_SUBTPMS_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"
#include "message.hpp"
#include "dsp_decimate.hpp"

#pragma GCC push_options
#pragma GCC optimize("Os")
#include "fprotos/subtpmsprotos.hpp"
#pragma GCC pop_options

#define OOK_EST_HIGH_RATIO 3
#define OOK_EST_LOW_RATIO 5
#define OOK_MAX_HIGH_LEVEL 450000

class SubTPMSProcessor : public BasebandProcessor {
   public:
    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    enum {
        STATE_IDLE = 0,
        STATE_PULSE = 1,
        STATE_GAP_START = 2,
        STATE_GAP = 3,
    } sig_state = STATE_IDLE;
    uint32_t low_estimate = 100;
    uint32_t high_estimate = 12000;
    uint32_t min_high_level = 10;
    uint32_t glitchDuration = 0;
    uint8_t numg = 0;
    size_t baseband_fs = 4'000'000;
    uint32_t nsPerDecSamp = 0;

    std::array<complex16_t, 512> dst{};
    const buffer_c16_t dst_buffer{
        dst.data(),
        dst.size()};

    dsp::decimate::FIRC8xR16x24FS4Decim4 decim_0{};
    dsp::decimate::FIRC16xR16x16Decim2 decim_1{};

    uint32_t currentDuration = 0;
    uint32_t threshold = 0x0630;
    bool currentHiLow = false;
    bool configured{false};

    // --- ÚJ, ADAPTÍV FM ÁLLAPOTTÉR ---
    struct DemodFMState {
        int16_t last_re_s = 0;
        int16_t last_im_s = 0;
        int32_t smoothed_discrim = 0;
        int32_t dc_offset = 0;
        int32_t deviation_avg = 0;
    };
    DemodFMState fm_state{};

    uint8_t modulation = 0;  // 0 am, 1 fm

    FProtoListGeneral* protoList = new SubTPMSProtos();
    void configure(const SubGhzFPRxConfigureMessage& message);

    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif /*__PROC_SUBTPMS_H__*/