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

#ifndef __UI_WAVC16_H__
#define __UI_WAVC16_H__

#include "ui.hpp"
#include "ui_language.hpp"
#include "ui_navigation.hpp"
#include "utility.hpp"
#include "file_path.hpp"
#include "ui_fileman.hpp"
#include "ui_receiver.hpp"
using namespace ui;

namespace ui::external_app::wavc16 {

class WavC16View : public View {
   public:
    WavC16View(NavigationView& nav);
    ~WavC16View();
    void open_file();
    bool convert_file(std::filesystem::path file);
    void focus() override;

    std::string title() const override {
        return "WavC16";
    };

    Button btn_open{{1 * 8, 2 * 8, 8 * 8, 16}, "Convert"};
    FrequencyField freq{{1 * 8, 0}};

   private:
    NavigationView& nav_;
    rf::Frequency freq_{433920000};
};
};  // namespace ui::external_app::wavc16

#endif /*__UI_WAVC16_H__*/
