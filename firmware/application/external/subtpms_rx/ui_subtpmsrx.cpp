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

#include "ui_subtpmsrx.hpp"
#include "audio.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "file_path.hpp"
#include "portapack_persistent_memory.hpp"

using namespace portapack;
using namespace ui;

namespace ui::external_app::subtpms_rx {

std::string SubTPMSRecentEntry::to_csv() {
    std::string csv = ";";
    csv += SubTPMSView::getSensorTypeName((FPROTO_SUBTPMS_SENSOR)sensorType);
    csv += ";" + to_string_dec_uint(bits) + ";";
    csv += to_string_hex(data, 64 / 4) + ";";
    csv += ";" + to_string_hex(id, 16 / 4) + ";" + to_string_dec_uint(battery) + ";" + to_string_dec_int(temperature) + ";" + to_string_decimal(pressure, 2);
    return csv;
}

void SubTPMSLogger::log_data(SubTPMSRecentEntry& data) {
    log_file.write_entry(data.to_csv());
}

void SubTPMSRecentEntryDetailView::update_data() {
    // set text elements
    text_type.set(SubTPMSView::getSensorTypeName((FPROTO_SUBTPMS_SENSOR)entry_.sensorType));

    text_id.set("0x" + to_string_hex(entry_.id));
    if (entry_.bits > 0) console.writeln("Bits: " + to_string_dec_uint(entry_.bits));
    if (entry_.data != 0) console.writeln("Data : " + to_string_hex(entry_.data));
    if (entry_.battery != 0xFF) console.writeln("Battery: " + to_string_dec_uint(entry_.battery));
    if (entry_.temperature != (int16_t)0xFFFF) console.writeln("Temperature: " + to_string_dec_int(entry_.temperature));
    if (entry_.pressure != -1.0) console.writeln("Pressure: " + to_string_decimal(entry_.pressure, 2));
}

SubTPMSRecentEntryDetailView::SubTPMSRecentEntryDetailView(NavigationView& nav, const SubTPMSRecentEntry& entry)
    : nav_{nav},
      entry_{entry} {
    add_children({&button_done,
                  &text_type,
                  &text_id,
                  &console,
                  &labels});

    button_done.on_select = [&nav](const ui::Button&) {
        nav.pop();
    };
    update_data();
}

void SubTPMSRecentEntryDetailView::focus() {
    button_done.focus();
}

void SubTPMSView::focus() {
    field_frequency.focus();
}

SubTPMSView::SubTPMSView(NavigationView& nav)
    : nav_{nav} {
    add_children({&rssi,
                  &channel,
                  &field_rf_amp,
                  &field_lna,
                  &field_vga,
                  &field_frequency,
                  &button_clear_list,
                  &check_log,
                  &labels,
                  &options_mode,
                  &recent_entries_view});

    baseband::run_prepared_image(portapack::memory::map::m4_code.base());
    logger = std::make_unique<SubTPMSLogger>();

    button_clear_list.on_select = [this](Button&) {
        recent.clear();
        recent_entries_view.set_dirty();
    };
    field_frequency.set_step(10000);
    check_log.on_select = [this](Checkbox&, bool v) {
        logging = v;
        if (logger && logging) {
            logger->append(logs_dir.string() + "/SubTPMSLOG_" + to_string_timestamp(rtc_time::now()) + ".CSV");
            logger->write_header();
        }
    };
    check_log.set_value(logging);
    const Rect content_rect{0, header_height, screen_width, screen_height - header_height};
    recent_entries_view.set_parent_rect(content_rect);
    recent_entries_view.on_select = [this](const SubTPMSRecentEntry& entry) {
        nav_.push<SubTPMSRecentEntryDetailView>(entry);
    };

    options_mode.on_change = [this](size_t, int32_t v) {
        modulation = v;
        chThdSleepMilliseconds(100);  // wait for the baseband thread to process the previous config, to avoid glitchy output when switching modes
        baseband::set_subghzd_config(modulation, receiver_model.sampling_rate());
    };
    signal_token_tick_second = rtc_time::signal_tick_second += [this]() {
        on_tick_second();
    };
    options_mode.set_selected_index(modulation, true);
    receiver_model.enable();
}

void SubTPMSView::on_tick_second() {
    for (auto& entry : recent) {
        entry.inc_age(1);
    }
    recent_entries_view.set_dirty();
}

void SubTPMSView::on_data(const SubTPMSDataMessage* data) {
    SubTPMSRecentEntry key{data->sensorType, data->data, data->bits, data->id, data->battery, data->temperature, data->pressure};
    if (logger && logging) {
        logger->log_data(key);
    }
    auto matching_recent = find(recent, key.key());
    if (matching_recent != std::end(recent)) {
        // Found within. Move to front of list, increment counter.
        (*matching_recent).reset_age();
        (*matching_recent).data = data->data;
        (*matching_recent).battery = data->battery;
        (*matching_recent).temperature = data->temperature;
        (*matching_recent).pressure = data->pressure;
        recent.push_front(*matching_recent);
        recent.erase(matching_recent);
    } else {
        recent.emplace_front(key);
        truncate_entries(recent, 64);
    }
    recent_entries_view.set_dirty();
}

SubTPMSView::~SubTPMSView() {
    rtc_time::signal_tick_second -= signal_token_tick_second;
    receiver_model.disable();
    baseband::shutdown();
}

const char* SubTPMSView::getSensorTypeName(FPROTO_SUBTPMS_SENSOR type) {
    switch (type) {
        case FPT_Schrader:
            return "Schrader";
        case FPT_Schrader_SMD3MA4:
            return "Sc_SMD3MA4";
        case FPT_Schrader_EG53MA4:
            return "Sc_EG53MA4";
        case FPT_Ford:
            return "Ford";
        case FPT_HyundaiVDO:
            return "Hyundai VDO";
        case FPT_Abarth124:
            return "Abarth 124";
        case FPT_Q85:
            return "Q85";
        case FPT_Airpuxem:
            return "Airpuxem";
        case FPT_AVE:
            return "AVE";
        case FPT_BMW:
            return "BMW";
        case FPT_AUDI:
            return "AUDI";
        case FPT_Citroen:
            return "Citroen";
        case FPT_Elantra2012:
            return "Elantra 2012";
        case FPT_Renault_0435R:
            return "Renault 0435R";
        case FPT_Toyota:
            return "Toyota";
        case FPT_Invalid:
        default:
            return "Unknown";
    }
}

std::string SubTPMSView::pad_string_with_spaces(int snakes) {
    std::string paddedStr(snakes, ' ');
    return paddedStr;
}

void SubTPMSView::on_freqchg(int64_t freq) {
    field_frequency.set_value(freq);
}

}  // namespace ui::external_app::subtpms_rx

namespace ui {

template <>
void RecentEntriesTable<ui::external_app::subtpms_rx::SubTPMSRecentEntries>::draw(
    const Entry& entry,
    const Rect& target_rect,
    Painter& painter,
    const Style& style,
    ui::RecentEntriesColumns& columns) {
    std::string line{};
    line.reserve(30);

    line = ui::external_app::subtpms_rx::SubTPMSView::getSensorTypeName((FPROTO_SUBTPMS_SENSOR)entry.sensorType);
    line = line + " " + to_string_hex(entry.id);
    line.resize(columns.at(0).second, ' ');
    std::string ageStr = to_string_dec_uint(entry.age < 999 ? entry.age : 999);
    std::string pressStr = (entry.pressure >= 0) ? to_string_decimal(entry.pressure, 1) : "";
    std::string tempStr = to_string_dec_int(entry.temperature);
    line += ui::external_app::subtpms_rx::SubTPMSView::pad_string_with_spaces(7 - pressStr.length()) + pressStr;
    line += ui::external_app::subtpms_rx::SubTPMSView::pad_string_with_spaces(6 - tempStr.length()) + tempStr;
    line += ui::external_app::subtpms_rx::SubTPMSView::pad_string_with_spaces(4 - ageStr.length()) + ageStr;
    line.resize(target_rect.width() / 8, ' ');
    painter.draw_string(target_rect.location(), style, line);
}

}  // namespace ui