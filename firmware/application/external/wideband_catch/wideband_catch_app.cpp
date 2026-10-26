#include "wideband_catch_app.hpp"
#include "baseband_api.hpp"
#include "portapack.hpp"
#include "string_format.hpp"
#include "audio.hpp"

using namespace portapack;

namespace ui::external_app::wideband_catch {

void CatchWindowWidget::update_state(bool is_locked, int32_t offset_hz, uint8_t width_bins, const std::array<uint8_t, 32>& mask) {
    locked = is_locked;
    freq_offset = offset_hz;
    width = width_bins;
    ignore_mask = mask;
    set_dirty();
}

void CatchWindowWidget::paint(Painter& painter) {
    const auto r = screen_rect();

    painter.fill_rectangle(r, Color::black());
    painter.draw_rectangle(r, Color::dark_grey());

    int center_x = r.left() + r.width() / 2;
    painter.draw_vline({center_x, r.top() + 1}, r.height() - 2, Color::grey());

    for (size_t bin = 0; bin < 256; bin++) {
        if (ignore_mask[bin >> 3] & (1 << (bin & 7))) {
            int signed_bin = (bin < 128) ? bin : (static_cast<int>(bin) - 256);
            int x = center_x + (signed_bin * (r.width() / 2)) / 128;
            painter.draw_vline({x, r.top() + 2}, r.height() - 4, Color::red());
        }
    }

    if (locked) {
        int x_offset = (freq_offset * (r.width() / 2)) / 768000;
        int lock_x = center_x + x_offset;
        int bar_w = std::max(3, static_cast<int>(width) * 2);

        Rect lock_rect{lock_x - bar_w / 2, r.top() + 2, bar_w, r.height() - 4};
        painter.fill_rectangle(lock_rect, Color::green());
    }
}

WidebandCatchView::WidebandCatchView(NavigationView& nav)
    : nav_(nav) {
    baseband::run_image(portapack::spi_flash::image_tag_wideband_catch);

    add_children({&labels,
                  &options_preset,
                  &field_frequency,
                  &options_mod,
                  &field_lna,
                  &field_vga,
                  &field_rf_amp,
                  &field_volume,
                  &field_threshold,
                  &field_hold,
                  &field_min_bins,
                  &field_max_bins,
                  &catch_window,
                  &text_status,
                  &text_caught_freq,
                  &text_signal_info,
                  &button_ignore,
                  &button_clear_ignore});

    receiver_model.set_sampling_rate(SAMPLE_RATE);
    receiver_model.set_baseband_bandwidth(1750000);
    receiver_model.enable();

    audio::output::start();

    field_threshold.set_value(35);
    field_hold.set_value(600);
    field_min_bins.set_value(1);
    field_max_bins.set_value(3);

    options_preset.on_change = [this](size_t, int32_t v) {
        ignore_freq = true;
        apply_preset(v);
    };

    field_frequency.on_change = [this](rf::Frequency f) {
        if (ignore_freq) {
            ignore_freq = false;
            return;
        }
        options_preset.set_selected_index(5);
        set_center_frequency(f);
    };
    field_frequency.on_edit = [this, &nav]() {
        auto new_view = nav.push<FrequencyKeypadView>(center_freq_);
        new_view->on_changed = [this](rf::Frequency f) {
            options_preset.set_selected_index(5);
            set_center_frequency(f);
        };
    };

    auto config_changed_cb = [this](int32_t) { send_config(); };
    options_mod.on_change = [this](size_t, int32_t) { send_config(); };
    field_threshold.on_change = config_changed_cb;
    field_hold.on_change = config_changed_cb;
    field_min_bins.on_change = config_changed_cb;
    field_max_bins.on_change = config_changed_cb;

    button_ignore.on_select = [this](Button&) {
        ignore_current_bin();
    };

    button_clear_ignore.on_select = [this](Button&) {
        clear_ignore_mask();
    };

    options_preset.set_selected_index(0);
    apply_preset(0);
}

WidebandCatchView::~WidebandCatchView() {
    audio::output::stop();
    receiver_model.disable();
    baseband::shutdown();
}

void WidebandCatchView::focus() {
    options_preset.focus();
}

void WidebandCatchView::apply_preset(int32_t preset_id) {
    switch (preset_id) {
        case 0:                                 // PMR446 All (446.0 - 446.2 MHz, cent: 446.1 MHz, NFM)
            options_mod.set_selected_index(0);  // NFM
            field_min_bins.set_value(1);
            field_max_bins.set_value(3);
            field_hold.set_value(800);
            set_center_frequency(446100000);
            break;

        case 1:                                 // LPD433 All (433.05 - 434.79 MHz, cent: 433.92 MHz, NFM)
            options_mod.set_selected_index(0);  // NFM
            field_min_bins.set_value(1);
            field_max_bins.set_value(3);
            field_hold.set_value(600);
            set_center_frequency(433920000);
            break;

        case 2:                                 // Airband Tower / Traffic (118.8 MHz +/- 768 kHz, AM)
            options_mod.set_selected_index(1);  // AM
            field_min_bins.set_value(1);
            field_max_bins.set_value(2);
            field_hold.set_value(1000);
            set_center_frequency(118800000);
            break;

        case 3:                                 // Airband Approach / Info (129.0 MHz +/- 768 kHz, AM)
            options_mod.set_selected_index(1);  // AM
            field_min_bins.set_value(1);
            field_max_bins.set_value(2);
            field_hold.set_value(1000);
            set_center_frequency(129000000);
            break;

        case 4:                                 // Marine VHF (: Ch 16 - 156.8 MHz cent, NFM)
            options_mod.set_selected_index(0);  // NFM
            field_min_bins.set_value(1);
            field_max_bins.set_value(4);  // Marine VHF  (25 kHz)
            field_hold.set_value(800);
            set_center_frequency(156500000);
            break;

        case 5:  // Manual / Custom
        default:
            break;
    }
    clear_ignore_mask();
}

void WidebandCatchView::set_center_frequency(rf::Frequency f) {
    center_freq_ = f;
    field_frequency.set_value(f);
    receiver_model.set_target_frequency(f);
}

void WidebandCatchView::send_config() {
    WidebandCatchConfigMessage msg{
        static_cast<uint8_t>(options_mod.selected_index_value()),
        static_cast<uint16_t>(field_threshold.value()),
        static_cast<uint8_t>(field_min_bins.value()),
        static_cast<uint8_t>(field_max_bins.value()),
        static_cast<uint16_t>(field_hold.value()),
        ignore_mask_};
    baseband::set_wideband_catch_config(msg);
}

void WidebandCatchView::on_status_message(const WidebandCatchStatusMessage& msg) {
    is_locked_ = msg.locked;

    if (msg.locked) {
        last_offset_hz_ = msg.freq_offset_hz;
        last_width_bins_ = msg.signal_width_bins;

        rf::Frequency caught_freq = center_freq_ + msg.freq_offset_hz;
        uint32_t mhz = caught_freq / 1000000;
        uint32_t khz_dec = (caught_freq % 1000000) / 100;  // 4  (100 Hz)

        text_status.set("STATE: >>> LOCKED AUDIO <<<");
        text_caught_freq.set("Caught: " + to_string_dec_uint(mhz) + "." + to_string_dec_uint(khz_dec, 4, '0') + " MHz");

        int32_t offset_khz = msg.freq_offset_hz / 1000;
        std::string tmp = (offset_khz >= 0 ? "+" : "") + to_string_dec_int(offset_khz);
        text_signal_info.set(
            "Offset: " + tmp +
            "k | Lvl: " + to_string_dec_uint(msg.signal_level));
    } else {
        text_status.set("STATE: SCANNING (1.536 MHz)...");
        text_signal_info.set("Offset: --- kHz | Level: ---");
    }

    catch_window.update_state(msg.locked, last_offset_hz_, last_width_bins_, ignore_mask_);
}

void WidebandCatchView::ignore_current_bin() {
    float bin_hz = static_cast<float>(SAMPLE_RATE) / static_cast<float>(FFT_SIZE);  // 6000 Hz
    int signed_bin = static_cast<int>(std::round(static_cast<float>(last_offset_hz_) / bin_hz));

    // -1, 0, +1 bin ignore, so that the interfering signal's sidebands don't block the search
    for (int d = -1; d <= 1; d++) {
        int b = (signed_bin + d + FFT_SIZE) % FFT_SIZE;
        ignore_mask_[b >> 3] |= (1 << (b & 7));
    }

    catch_window.update_state(false, 0, 0, ignore_mask_);
    send_config();
}

void WidebandCatchView::clear_ignore_mask() {
    ignore_mask_.fill(0);
#ifndef PRALINE
    // It's recommended to mask the central DC bin (0 Hz) by default
    ignore_mask_[0] |= 0b00000011;   // bin 0, 1
    ignore_mask_[31] |= 0b10000000;  // bin 255
#endif
    catch_window.update_state(is_locked_, last_offset_hz_, last_width_bins_, ignore_mask_);
    send_config();
}

}  // namespace ui::external_app::wideband_catch
