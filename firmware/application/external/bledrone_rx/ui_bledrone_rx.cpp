#include "ui_bledrone_rx.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "audio.hpp"
#include "portapack_persistent_memory.hpp"
#include <cmath>

using namespace portapack;

namespace ui {

template <>
void RecentEntriesTable<ui::external_app::bledrone_rx::DroneRecentEntries>::draw(
    const Entry& entry,
    const Rect& target_rect,
    Painter& painter,
    const Style& style,
    RecentEntriesColumns& columns) {
    Color target_color;
    std::string entry_string;

    // Szin-kodolas a kor alapjan (Mint az ADSB-nel)
    switch (entry.state) {
        case ui::external_app::bledrone_rx::DroneAgeState::Current:
            target_color = Theme::getInstance()->fg_green->foreground;
            break;
        case ui::external_app::bledrone_rx::DroneAgeState::Recent:
            target_color = Theme::getInstance()->fg_light->foreground;
            break;
        default:
            target_color = Theme::getInstance()->fg_medium->foreground;
    };

    // ID vagy MAC megjenitese
    std::string id_str = entry.uas_id.empty() ? entry.mac_str : entry.uas_id;
    uint8_t firstcolwidth = columns.at(0).second;
    id_str.resize(firstcolwidth, ' ');

    entry_string += id_str;

    // Alt
    if (entry.has_loc) {
        entry_string += to_string_dec_int(entry.alt, 4) + " ";
    } else {
        entry_string += "   - ";
    }

    // RSSI
    entry_string += to_string_dec_int(entry.rssi, 3) + " ";

    // Hits
    entry_string += (entry.hits <= 999 ? to_string_dec_uint(entry.hits, 3) + " " : "1k+ ");

    // Age
    entry_string += to_string_dec_uint(entry.age, 3);

    // Kirajzolas az arnyekolassal egyutt
    painter.draw_string(target_rect.location(), style, entry_string);
    // painter.draw_string(target_rect.location(), {target_color, style.background}, entry_string);
}

}  // namespace ui

namespace ui::external_app::bledrone_rx {

DroneDetailsView::DroneDetailsView(NavigationView& nav, const DroneRecentEntry& entry)
    : entry_(entry), nav_(nav) {
    add_children({&labels, &text_mac, &text_id, &text_d_loc, &text_d_alt, &text_p_loc, &text_hits, &button_see_map});

    button_see_map.on_select = [this, &nav](Button&) {
        if (entry_.has_loc) {
            geomap_view_ = nav.push<GeoMapView>(
                entry_.uas_id.empty() ? entry_.mac_str : entry_.uas_id,
                entry_.alt,
                GeoPos::alt_unit::METERS,
                GeoPos::spd_unit::KMPH,
                entry_.lat,
                entry_.lon,
                0  // A drón irányszöge ritkán ismert a legacy csomagokban
            );
            nav.set_on_pop([this]() {
                geomap_view_ = nullptr;
                refresh_ui();
            });
        }
    };

    refresh_ui();
}

std::string DroneDetailsView::format_latlon(int32_t raw_coord) {
    int32_t deg = raw_coord / 10000000;
    int32_t frac = std::abs(raw_coord) % 10000000;
    return to_string_dec_int(deg) + "." + to_string_dec_uint(frac, 7, '0');
}

void DroneDetailsView::refresh_ui() {
    text_mac.set(entry_.mac_str);
    text_id.set(entry_.uas_id.empty() ? "N/A" : entry_.uas_id);

    if (entry_.has_loc) {
        text_d_loc.set(format_latlon(entry_.lat) + ", " + format_latlon(entry_.lon));
        text_d_alt.set(to_string_dec_int(entry_.alt) + " m");
    }

    if (entry_.has_pilot) {
        text_p_loc.set(format_latlon(entry_.p_lat) + ", " + format_latlon(entry_.p_lon));
    }

    text_hits.set(to_string_dec_uint(entry_.hits));
}

void DroneDetailsView::update(const DroneRecentEntry& entry) {
    entry_ = entry;

    if (geomap_view_) {
        geomap_view_->update_position(entry.lat, entry.lon, 0, entry.alt, 0);
    } else {
        refresh_ui();
    }
}

void DroneDetailsView::focus() {
    button_see_map.focus();
}

BleDroneRxView::BleDroneRxView(NavigationView& nav) : nav_{nav} {
    // Futtatjuk a BTLERx bázissáv processzort
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&labels_top, &field_lna, &field_vga, &options_channel, &recent_entries_view});

    recent_entries_view.set_parent_rect({0, 16, screen_width, UI_POS_HEIGHT_REMAINING(1)});
    recent_entries_view.on_select = [this, &nav](const DroneRecentEntry& entry) {
        detail_key = entry.key();
        details_view = nav.push<DroneDetailsView>(entry);

        nav.set_on_pop([this]() {
            detail_key = DroneRecentEntry::invalid_key;
            details_view = nullptr;
        });
    };

    signal_token_tick_second = rtc_time::signal_tick_second += [this]() {
        on_tick_second();
    };

    receiver_model.set_sampling_rate(4000000);
    receiver_model.set_baseband_bandwidth(2000000);
    receiver_model.set_modulation(ReceiverModel::Mode::WidebandFMAudio);
    receiver_model.enable();

    options_channel.on_change = [this](size_t, int32_t v) { this->on_channel_changed(v); };
    options_channel.set_selected_index(0);
    on_channel_changed(37);

    if (persistent_memory::beep_on_packets()) {
        audio::set_rate(audio::Rate::Hz_24000);
        audio::output::start();
    }
}

void BleDroneRxView::on_channel_changed(int32_t channel) {
    uint32_t freq = 2402000000;
    if (channel == 38)
        freq = 2426000000;
    else if (channel == 39)
        freq = 2480000000;

    receiver_model.set_target_frequency(freq);
    baseband::set_btlerx(channel);
}

void BleDroneRxView::on_tick_second() {
    update_recent_entries();
    refresh_ui();
}

void BleDroneRxView::update_recent_entries() {
    for (auto& entry : recent) {
        entry.inc_age(1);
    }

    recent.sort([](const auto& left, const auto& right) {
        return left.state < right.state;
    });

    auto it = recent.rbegin();
    while (it != recent.rend() && it->state == DroneAgeState::Expired) {
        std::advance(it, 1);
    }
    recent.erase(it.base(), recent.end());
}

void BleDroneRxView::refresh_ui() {
    if (details_view) {
        for (const auto& entry : recent) {
            if (entry.key() == detail_key) {
                details_view->update(entry);
                break;
            }
        }
    } else {
        recent_entries_view.set_dirty();
    }
}

void BleDroneRxView::focus() {
    options_channel.focus();
}

DroneRecentEntry& BleDroneRxView::find_or_create_entry(uint32_t mac_key) {
    auto it = find(recent, mac_key);
    if (it != recent.end())
        return *it;
    return recent.emplace_front(mac_key);
}

void BleDroneRxView::on_packet(const BLEPacketMessage* msg) {
    auto pkt = msg->packet;
    if (!pkt || pkt->size == 0) return;

    uint8_t idx = 0;
    bool is_drone = false;
    uint8_t remote_id_payload_idx = 0;
    uint8_t remote_id_payload_len = 0;

    // BLE AD Payload villámgyors pásztázása
    while (idx < pkt->dataLen) {
        uint8_t adv_len = pkt->data[idx++];
        if (adv_len == 0 || idx + adv_len - 1 > pkt->dataLen) break;

        uint8_t adv_type = pkt->data[idx++];

        // 0x16 Service Data típus ellenőrzése
        if (adv_type == 0x16 && adv_len >= 4) {
            uint16_t uuid = pkt->data[idx] | (pkt->data[idx + 1] << 8);
            if (uuid == 0xFA0B) {  // ASTM Remote ID Service UUID
                is_drone = true;
                remote_id_payload_idx = idx + 2;      // Az UUID (2 bájt) után kezdődik az adat
                remote_id_payload_len = adv_len - 3;  // adv_len tartalmazza az adv_type-ot és az UUID-t is
                break;
            }
        }
        idx += (adv_len - 1);
    }

    // Ha ez nem drón, azonnal eldobjuk a csomagot! (Nem szemeteljük tele a UI-t)
    if (!is_drone) return;

    // MAC kulcs generalasa (also 4 bajt azonositasra a legalkalmasabb)
    uint32_t mac_key = (pkt->macAddress[2] << 24) | (pkt->macAddress[3] << 16) |
                       (pkt->macAddress[4] << 8) | pkt->macAddress[5];

    auto& entry = find_or_create_entry(mac_key);

    entry.inc_hit();
    entry.reset_age();
    entry.rssi = pkt->max_dB;

    if (entry.mac_str.empty()) {
        entry.mac_str = to_string_hex(pkt->macAddress[5], 2) + ":" +
                        to_string_hex(pkt->macAddress[4], 2) + ":" +
                        to_string_hex(pkt->macAddress[3], 2) + ":" +
                        to_string_hex(pkt->macAddress[2], 2) + ":" +
                        to_string_hex(pkt->macAddress[1], 2) + ":" +
                        to_string_hex(pkt->macAddress[0], 2);
    }

    // Remote ID adatok tényleges dekódolása
    parse_remote_id(&pkt->data[remote_id_payload_idx], remote_id_payload_len, entry);

    if (persistent_memory::beep_on_packets()) {
        baseband::request_audio_beep(1000, 24000, 30);
    }
}

void BleDroneRxView::parse_remote_id(const uint8_t* payload, uint8_t len, DroneRecentEntry& entry) {
    if (len < 1) return;
    uint8_t msg_type = payload[0] & 0x0F;

    if (msg_type == 0x00 && len >= 21) {
        // Message Type 0: Basic ID (Sorozatszam)
        char drone_id[21];
        for (int i = 0; i < 20; i++) {
            char c = payload[i + 2];
            drone_id[i] = (c >= 32 && c <= 126) ? c : ' ';
        }
        drone_id[20] = '\0';
        entry.uas_id = std::string(drone_id);
    } else if (msg_type == 0x01 && len >= 17) {
        // Message Type 1: Location (Geographic Position es Altitude)
        int16_t alt_raw = payload[4] | (payload[5] << 8);
        if (alt_raw != (int16_t)0xFFFF) {
            entry.alt = (alt_raw * 5) / 10 - 1000;
        }

        entry.lat = payload[8] | (payload[9] << 8) | (payload[10] << 16) | (payload[11] << 24);
        entry.lon = payload[12] | (payload[13] << 8) | (payload[14] << 16) | (payload[15] << 24);
        entry.has_loc = true;
    } else if (msg_type == 0x04 && len >= 17) {
        // Message Type 4: System (Pilot Position)
        entry.p_lat = payload[4] | (payload[5] << 8) | (payload[6] << 16) | (payload[7] << 24);
        entry.p_lon = payload[8] | (payload[9] << 8) | (payload[10] << 16) | (payload[11] << 24);
        entry.has_pilot = true;
    }
}

BleDroneRxView::~BleDroneRxView() {
    rtc_time::signal_tick_second -= signal_token_tick_second;
    audio::output::stop();
    receiver_model.disable();
    baseband::shutdown();
}

}  // namespace ui::external_app::bledrone_rx