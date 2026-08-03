

#include "ui.hpp"
#include "ui_bledrone_rx.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::bledrone_rx {
void initialize_app(ui::NavigationView& nav) {
    nav.push<BleDroneRxView>();
}
}  // namespace ui::external_app::bledrone_rx

extern "C" {

__attribute__((section(".external_app.app_bledrone_rx.application_information"), used)) application_information_t _application_information_bledrone_rx = {
    (uint8_t*)0x00000000,
    ui::external_app::bledrone_rx::initialize_app,
    CURRENT_HEADER_VERSION,
    VERSION_MD5,

    "BleDrone",
    /*.bitmap_data = */ {
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0xF8,
        0x1F,
        0x04,
        0x20,
        0x02,
        0x40,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xAB,
        0xDF,
        0xAB,
        0xDF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    },
    /*.icon_color = */ ui::Color::yellow().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = portapack::spi_flash::image_tag_btle_rx */ {'P', 'B', 'T', 'R'},
    /*.m4_app_offset = */ 0x00000000,  // will be filled at compile time
};
}
