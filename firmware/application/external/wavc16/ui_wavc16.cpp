/*
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

#include "ui_wavc16.hpp"

#include "rtc_time.hpp"
#include "string_format.hpp"
#include "file_path.hpp"
#include "metadata_file.hpp"
#include "flipper_subfile.hpp"
#include "portapack_persistent_memory.hpp"
#include "io_wave.hpp"
using namespace portapack;
using namespace ui;

namespace ui::external_app::wavc16 {

void WavC16View::focus() {
    btn_open.focus();
}

WavC16View::WavC16View(NavigationView& nav)
    : nav_{nav} {
    add_children({&btn_open, &freq});

    freq.set_value(433920000);
    freq.on_change = [this](rf::Frequency f) {
        freq_ = f;
    };
    btn_open.on_select = [this, &nav](Button&) {
        open_file();
    };
}

WavC16View::~WavC16View() {
}

uint8_t WavC16View::convert_file(std::filesystem::path file) {
    auto reader = std::make_unique<WAVFileReader>();
    if (reader->open(file)) {
        if ((reader->channels() == 2) || (reader->channels() == 1)) {                    // todo check if it is really good
            std::filesystem::path txt_file = file.filename().replace_extension(".txt");  // center_frequency=nnnnn \r\n sample_rate=nnnnn
            std::filesystem::path c16_file = file.filename().replace_extension(".c16");
            File f;
            size_t length;
            StringFormatBuffer buffer;
            // create txt file
            auto error = f.create(captures_dir + "/" + txt_file);
            if (error) return 5;

            f.write("center_frequency=", 17);
            f.write(to_string_dec_int(freq_, buffer, length), length);
            f.write("\r\n", 2);
            f.write("sample_rate=", 12);
            f.write(to_string_dec_uint(reader->sample_rate(), buffer, length), length);
            f.write("\r\n", 2);
            f.close();
            // create c16 file
            error = f.create(captures_dir + "/" + c16_file);
            if (error) return 4;

            uint8_t bits_per_sample = reader->bits_per_sample();
            for (size_t i = 0; i < reader->sample_count(); i++) {
                reader->data_seek(i);
                int16_t out = 0;
                if (bits_per_sample == 8) {
                    int8_t sample;
                    reader->read(&sample, 1);
                    out = sample << 8;
                    f.write(&out, 2);
                } else if (bits_per_sample == 16) {
                    int16_t sample;
                    reader->read(&sample, 2);
                    f.write(&out, 2);
                } else if (bits_per_sample == 24) {
                    int32_t sample;
                    reader->read(&sample, 3);
                    out = sample >> 8;
                    f.write(&out, 2);
                } else if (bits_per_sample == 32) {
                    int32_t sample;
                    reader->read(&sample, 4);
                    out = sample >> 16;
                    f.write(&out, 2);
                } else {
                    return 2;
                }
            }
            f.close();
        } else {
            return 3;
        }
        return 0;
    }
    return 1;
}

void WavC16View::open_file() {
    auto open_view = nav_.push<FileLoadView>(".WAV");
    open_view->push_dir(wav_dir);
    open_view->on_changed = [this](std::filesystem::path new_file_path) {
        uint8_t ret = convert_file(new_file_path);
        if (ret == 0) {
            btn_open.set_text("OK");
            // nav_.display_modal("Convert", "Convert done");
        } else {
            // nav_.display_modal("Error", "Can't convert WAV file");
            btn_open.set_text(to_string_dec_uint(ret));
        }
    };
}

}  // namespace ui::external_app::wavc16