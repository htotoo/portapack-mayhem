#ifndef __UI_BLEDRONE_RX_H__
#define __UI_BLEDRONE_RX_H__

#include "ui.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "recent_entries.hpp"  // A kert recent_entries hivatkozas
#include "ui_geomap.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "message.hpp"
#include "rtc_time.hpp"
#include <string>

namespace ui::external_app::bledrone_rx {

struct DroneAgeLimit {
    static constexpr int Current = 5;
    static constexpr int Recent = 20;
    static constexpr int Expired = 120;
};

enum class DroneAgeState : uint8_t {
    Invalid,
    Current,
    Recent,
    Old,
    Expired,
};

struct DroneRecentEntry {
    using Key = uint32_t;
    static constexpr Key invalid_key = 0xffffffff;

    uint32_t mac_key{};  // A MAC also 4 bajtja azonosotonak
    std::string mac_str{};
    std::string uas_id{};

    int32_t lat{0}, lon{0}, alt{0};
    int32_t p_lat{0}, p_lon{0};

    bool has_loc{false};
    bool has_pilot{false};

    uint16_t hits{0};
    uint32_t age{0};
    int32_t rssi{0};

    DroneAgeState state{DroneAgeState::Invalid};

    DroneRecentEntry(const uint32_t key) : mac_key{key} {}

    Key key() const { return mac_key; }

    void inc_hit() { hits++; }
    void reset_age() { age = 0; }

    void inc_age(int delta) {
        age += delta;
        if (age < DroneAgeLimit::Current)
            state = DroneAgeState::Current;
        else if (age < DroneAgeLimit::Recent)
            state = DroneAgeState::Recent;
        else if (age < DroneAgeLimit::Expired)
            state = DroneAgeState::Old;
        else
            state = DroneAgeState::Expired;
    }
};

using DroneRecentEntries = RecentEntries<DroneRecentEntry>;

class DroneDetailsView : public View {
   public:
    DroneDetailsView(NavigationView& nav, const DroneRecentEntry& entry);
    void focus() override;
    std::string title() const override { return "Drone Details"; }
    void update(const DroneRecentEntry& entry);

   private:
    DroneRecentEntry entry_{DroneRecentEntry::invalid_key};
    NavigationView& nav_;
    GeoMapView* geomap_view_{nullptr};

    void refresh_ui();
    std::string format_latlon(int32_t raw_coord);

    Labels labels{
        {{0, UI_POS_Y(0)}, "MAC  :", Theme::getInstance()->fg_light->foreground},
        {{0, UI_POS_Y(1)}, "ID   :", Theme::getInstance()->fg_light->foreground},
        {{0, UI_POS_Y(3)}, "D.Loc:", Theme::getInstance()->fg_light->foreground},
        {{0, UI_POS_Y(4)}, "D.Alt:", Theme::getInstance()->fg_light->foreground},
        {{0, UI_POS_Y(6)}, "P.Loc:", Theme::getInstance()->fg_light->foreground},
        {{0, UI_POS_Y(8)}, "Hits :", Theme::getInstance()->fg_light->foreground}};

    Text text_mac{{7 * 8, UI_POS_Y(0), 16 * 8, 16}, "-"};
    Text text_id{{7 * 8, UI_POS_Y(1), 20 * 8, 16}, "-"};

    Text text_d_loc{{7 * 8, UI_POS_Y(3), 22 * 8, 16}, "-"};
    Text text_d_alt{{7 * 8, UI_POS_Y(4), 16 * 8, 16}, "-"};

    Text text_p_loc{{7 * 8, UI_POS_Y(6), 22 * 8, 16}, "-"};

    Text text_hits{{7 * 8, UI_POS_Y(8), 16 * 8, 16}, "-"};

    Button button_see_map{
        {UI_POS_X_CENTER(12), UI_POS_Y(11), UI_POS_WIDTH(12), UI_POS_HEIGHT(3)},
        "See on map"};
};

class BleDroneRxView : public View {
   public:
    BleDroneRxView(NavigationView& nav);
    ~BleDroneRxView();
    void focus() override;

    std::string title() const override { return "BLE Drone ID Scanner"; };

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{};
    app_settings::SettingsManager settings_{"rx_bledrone", app_settings::Mode::RX};

    Labels labels_top{
        {{0, 0}, "LNA:   VGA:   CH:", Theme::getInstance()->fg_light->foreground}};

    LNAGainField field_lna{{4 * 8, 0}};
    VGAGainField field_vga{{11 * 8, 0}};

    OptionsField options_channel{
        {18 * 8, 0},
        8,
        {{"37(2402)", 37}, {"38(2426)", 38}, {"39(2480)", 39}}};

    RecentEntriesColumns columns{
        {{"MAC / UAS ID", 15}, {"Alt", 4}, {"RSSI", 4}, {"Hit", 3}, {"Age", 3}}};
    DroneRecentEntries recent{};
    RecentEntriesView<DroneRecentEntries> recent_entries_view{columns, recent};

    DroneRecentEntry::Key detail_key{DroneRecentEntry::invalid_key};
    DroneDetailsView* details_view{nullptr};

    SignalToken signal_token_tick_second{};

    void on_channel_changed(int32_t channel);
    void on_packet(const BLEPacketMessage* message);
    void parse_remote_id(const uint8_t* payload, uint8_t len, DroneRecentEntry& entry);

    void on_tick_second();
    void update_recent_entries();
    void refresh_ui();
    DroneRecentEntry& find_or_create_entry(uint32_t mac_key);

    MessageHandlerRegistration message_handler_packet{
        Message::ID::BlePacket,
        [this](Message* const p) {
            this->on_packet(static_cast<const BLEPacketMessage*>(p));
        }};
};

}  // namespace ui::external_app::bledrone_rx

#endif  // __UI_BLEDRONE_RX_H__