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

#ifndef __UI_SubTPMS_H__
#define __UI_SubTPMS_H__

#define SD_NO_SERIAL 0xFFFFFFFF
#define SD_NO_BTN 0xFF
#define SD_NO_CNT 0xFF

#include "ui.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "utility.hpp"
#include "log_file.hpp"
#include "recent_entries.hpp"

#include "../../baseband/fprotos/subtpmstypes.hpp"

using namespace ui;

namespace ui::external_app::subtpms_rx {

struct SubTPMSRecentEntry {
    using Key = uint64_t;
    static constexpr Key invalid_key = 0x0fffffff;
    uint8_t sensorType = FPT_Invalid;
    uint16_t bits = 0;
    uint16_t age = 0;  // updated on each seconds, show how long the signal was last seen
    uint64_t data = 0;
    uint32_t id = 0xFFFFFFFF;
    uint8_t battery = 0xFF;
    int16_t temperature = 0xFFFF;
    float pressure = -1.0;

    SubTPMSRecentEntry() {}
    SubTPMSRecentEntry(
        uint8_t sensorType,
        uint64_t data = 0,
        uint16_t bits = 0,
        uint32_t id = 0xFFFFFFFF,
        uint8_t battery = 0xFF,
        int16_t temperature = 0xFFFF,
        float pressure = -1.0)
        : sensorType{sensorType},
          bits{bits},
          data{data},
          id{id},
          battery{battery},
          temperature{temperature},
          pressure{pressure} {
    }
    Key key() const {
        return ((static_cast<uint64_t>(id) ^ ((static_cast<uint64_t>(sensorType) & 0xFF) << 0)));  // should be optimized...
    }
    void inc_age(int delta) {
        if (UINT16_MAX - delta > age) age += delta;
    }
    void reset_age() {
        age = 0;
    }

    std::string to_csv();
};

class SubTPMSLogger {
   public:
    Optional<File::Error> append(const std::filesystem::path& filename) {
        return log_file.append(filename);
    }

    void log_data(SubTPMSRecentEntry& data);
    void write_header() {
        log_file.write_entry(";Type; Bits; Data;");
    }

   private:
    LogFile log_file{};
};
using SubTPMSRecentEntries = RecentEntries<SubTPMSRecentEntry>;
using SubTPMSRecentEntriesView = RecentEntriesView<SubTPMSRecentEntries>;

class SubTPMSView : public View {
   public:
    SubTPMSView(NavigationView& nav);
    ~SubTPMSView();

    void focus() override;

    std::string title() const override { return "SubTPMS"; };
    static const char* getSensorTypeName(FPROTO_SUBTPMS_SENSOR type);
    static std::string pad_string_with_spaces(int snakes);

   private:
    void on_tick_second();
    void on_data(const SubTPMSDataMessage* data);

    NavigationView& nav_;
    RxRadioState radio_state_{
        433'920'000 /* frequency */,
        1'750'000 /* bandwidth */,
        4'000'000 /* sampling rate */,
        ReceiverModel::Mode::AMAudio};
    bool logging = false;
    uint8_t modulation = 0;
    app_settings::SettingsManager settings_{
        "rx_subtpms"sv,
        app_settings::Mode::RX,
        {
            {"log"sv, &logging},
            {"modulationmode"sv, &modulation},
        }};

    SubTPMSRecentEntries recent{};

    RFAmpField field_rf_amp{
        {UI_POS_X(13), UI_POS_Y(0)}};
    LNAGainField field_lna{
        {UI_POS_X(15), UI_POS_Y(0)}};
    VGAGainField field_vga{
        {UI_POS_X(18), UI_POS_Y(0)}};
    RSSI rssi{
        {UI_POS_X(21), 0, UI_POS_WIDTH_REMAINING(24), 4}};
    Channel channel{
        {UI_POS_X(21), 5, UI_POS_WIDTH_REMAINING(24), 4},
    };
    RxFrequencyField field_frequency{
        {UI_POS_X(0), UI_POS_Y(0)},
        nav_};

    SignalToken signal_token_tick_second{};

    Button button_clear_list{
        {UI_POS_X(0), UI_POS_Y(1), UI_POS_WIDTH(7), UI_POS_HEIGHT(2)},
        "Clear"};

    Checkbox check_log{
        {UI_POS_X(8), UI_POS_Y(1)},
        3,
        "Log",
        true};

    Labels labels{
        {{UI_POS_X(15), UI_POS_Y(1)}, "Mode:", Theme::getInstance()->fg_light->foreground},
    };
    ui::OptionsField options_mode{
        {UI_POS_X(22), UI_POS_Y(1)},
        3,
        {{"AM", 0}, {"FM", 1}}};

    static constexpr auto header_height = 3 * 16;

    std::unique_ptr<SubTPMSLogger> logger{};

    ui::RecentEntriesColumns columns{{
        {"Type", 0},
        {"Press", 6},
        {"Temp", 5},
        {"Age", 3},
    }};
    SubTPMSRecentEntriesView recent_entries_view{columns, recent};

    void on_freqchg(int64_t freq);
    MessageHandlerRegistration message_handler_freqchg{
        Message::ID::FreqChangeCommand,
        [this](Message* const p) {
            const auto message = static_cast<const FreqChangeCommandMessage*>(p);
            this->on_freqchg(message->freq);
        }};

    MessageHandlerRegistration message_handler_packet{
        Message::ID::SubTPMSData,
        [this](Message* const p) {
            const auto message = static_cast<const SubTPMSDataMessage*>(p);
            this->on_data(message);
        }};
};

class SubTPMSRecentEntryDetailView : public View {
   public:
    SubTPMSRecentEntryDetailView(NavigationView& nav, const SubTPMSRecentEntry& entry);

    void update_data();
    void focus() override;

   private:
    NavigationView& nav_;
    SubTPMSRecentEntry entry_{};

    uint32_t serial = 0;
    std::string btn = "";
    uint32_t cnt = SD_NO_CNT;

    Text text_type{{UI_POS_X(0), UI_POS_Y(1), UI_POS_WIDTH(15), UI_POS_HEIGHT(1)}, "?"};
    Text text_id{{UI_POS_X(6), UI_POS_Y(2), UI_POS_WIDTH(10), UI_POS_HEIGHT(1)}, "?"};

    Console console{
        {UI_POS_X(0), UI_POS_Y(4), UI_POS_MAXWIDTH, screen_height - (4 * 16) - 36}};

    Labels labels{
        {{UI_POS_X(0), UI_POS_Y(0)}, "Type:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(2)}, "Serial: ", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(3)}, "Data:", Theme::getInstance()->fg_light->foreground},
    };

    Button button_done{
        {screen_width - 96 - 4, screen_height - 32 - 12, 96, 32},
        "Done"};
};

}  // namespace ui::external_app::subtpms_rx

#endif /*__UI_SubTPMSRX_H__*/
