#ifndef __WIDEBAND_CATCH_APP_H__
#define __WIDEBAND_CATCH_APP_H__

#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "ui_rssi.hpp"
#include "ui_channel.hpp"
#include "message.hpp"
#include "receiver_model.hpp"
#include "radio.hpp"

#include <array>

namespace ui::external_app::wideband_catch {

class CatchWindowWidget : public Widget {
   public:
    CatchWindowWidget(Rect parent_rect) : Widget(parent_rect) {}

    void paint(Painter& painter) override;
    void update_state(bool is_locked, int32_t offset_hz, uint8_t width_bins, const std::array<uint8_t, 32>& mask);

   private:
    bool locked{false};
    int32_t freq_offset{0};
    uint8_t width{1};
    std::array<uint8_t, 32> ignore_mask{};
};

class WidebandCatchView : public View {
   public:
    WidebandCatchView(NavigationView& nav);
    ~WidebandCatchView();

    void focus() override;
    std::string title() const override { return "WB Catcher"; }

   private:
    NavigationView& nav_;

    static constexpr uint32_t SAMPLE_RATE = 1536000;
    static constexpr size_t FFT_SIZE = 256;

    rf::Frequency center_freq_{446100000};
    bool is_locked_{false};
    int32_t last_offset_hz_{0};
    uint8_t last_width_bins_{0};
    std::array<uint8_t, 32> ignore_mask_{};

    bool ignore_freq = false;

    Labels labels{
        {{UI_POS_X(0), UI_POS_Y(0)}, "Preset:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(1)}, "Center:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(19), UI_POS_Y(1)}, "Mod:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(2)}, "LNA:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(9), UI_POS_Y(2)}, "VGA:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(19), UI_POS_Y(2)}, "AMP:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(4)}, "Thresh:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(14), UI_POS_Y(4)}, "Hold(ms):", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(5)}, "MinBin:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(14), UI_POS_Y(5)}, "MaxBin:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(7)}, "Window (-768k .. 0 .. +768k):", Theme::getInstance()->fg_medium->foreground}};

    OptionsField options_preset{
        {UI_POS_X(7), UI_POS_Y(0)},
        22,
        {{"PMR446 All (446.1 MHz)", 0},
         {"LPD433 All (433.9 MHz)", 1},
         {"Airband Twr (118.8 MHz)", 2},
         {"Airband App (129.0 MHz)", 3},
         {"Marine VHF (156.8 MHz)", 4},
         {"Manual / Custom", 5}}};

    FrequencyField field_frequency{
        {UI_POS_X(7), UI_POS_Y(1)}};

    OptionsField options_mod{
        {UI_POS_X(24), UI_POS_Y(1)},
        4,
        {{"NFM", 0},
         {"AM ", 1}}};

    LNAGainField field_lna{
        {UI_POS_X(4), UI_POS_Y(2)}};

    VGAGainField field_vga{
        {UI_POS_X(13), UI_POS_Y(2)}};

    RFAmpField field_rf_amp{
        {UI_POS_X(24), UI_POS_Y(2)}};

    AudioVolumeField field_volume{
        {UI_POS_X(27), UI_POS_Y(2)}};

    NumberField field_threshold{
        {UI_POS_X(8), UI_POS_Y(4)},
        3,
        {5, 255},
        5,
        ' '};

    NumberField field_hold{
        {UI_POS_X(24), UI_POS_Y(4)},
        4,
        {100, 3000},
        100,
        ' '};

    NumberField field_min_bins{
        {UI_POS_X(8), UI_POS_Y(5)},
        2,
        {1, 16},
        1,
        ' '};

    NumberField field_max_bins{
        {UI_POS_X(24), UI_POS_Y(5)},
        2,
        {1, 32},
        1,
        ' '};

    CatchWindowWidget catch_window{
        {UI_POS_X(0), UI_POS_Y(8), UI_POS_WIDTH(30), UI_POS_HEIGHT(2)}};

    Text text_status{
        {UI_POS_X(0), UI_POS_Y(11), UI_POS_WIDTH(30), UI_POS_HEIGHT(1)},
        "STATE: SCANNING..."};

    Text text_caught_freq{
        {UI_POS_X(0), UI_POS_Y(12), UI_POS_WIDTH(30), UI_POS_HEIGHT(1)},
        "Caught: ---.---- MHz"};

    Text text_signal_info{
        {UI_POS_X(0), UI_POS_Y(13), UI_POS_WIDTH(30), UI_POS_HEIGHT(1)},
        "Offset: --- kHz | Level: ---"};

    Button button_ignore{
        {UI_POS_X(0), UI_POS_Y(15), UI_POS_WIDTH(14), UI_POS_HEIGHT(2)},
        "Ignore Freq"};

    Button button_clear_ignore{
        {UI_POS_X(16), UI_POS_Y(15), UI_POS_WIDTH(14), UI_POS_HEIGHT(2)},
        "Clear Ignore"};

    MessageHandlerRegistration message_handler_status{
        Message::ID::WidebandCatchStatus,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const WidebandCatchStatusMessage*>(p);
            this->on_status_message(message);
        }};

    void apply_preset(int32_t preset_id);
    void send_config();
    void set_center_frequency(rf::Frequency f);
    void on_status_message(const WidebandCatchStatusMessage& msg);
    void ignore_current_bin();
    void clear_ignore_mask();
};

}  // namespace ui::external_app::wideband_catch

#endif /* __WIDEBAND_CATCH_APP_H__ */