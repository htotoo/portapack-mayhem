#include "ui_rds_rx.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "portapack_persistent_memory.hpp"
#include <cstring>

using namespace portapack;

namespace ui::external_app::rds_rx {

RdsRxView::RdsRxView(NavigationView& nav) : nav_{nav} {
    // DSP (Baseband) firmware elindítása
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&rssi, &channel, &field_rf_amp, &field_lna, &field_vga,
                  &field_frequency, &text_pi, &text_pty, &text_debug,
                  &text_ps_label, &text_ps_name, &console});

    field_frequency.set_step(100000);  // 100 kHz lépésköz a normál FM sávhoz

    // Vevő paramétereinek beállítása az RDS/WBFM-hez
    receiver_model.set_modulation(ReceiverModel::Mode::WidebandFMAudio);
    receiver_model.set_sampling_rate(3072000);       // Alap sávszélesség
    receiver_model.set_baseband_bandwidth(1750000);  // Bőséges sáv az 57kHz MPX-hez
    receiver_model.set_squelch_level(0);
    receiver_model.enable();

    // Pufferek inicializálása
    memset(ps_name, ' ', 8);
    ps_name[8] = '\0';
    memset(radio_text, ' ', 64);
    radio_text[64] = '\0';
}

void RdsRxView::focus() {
    field_frequency.focus();
}

void RdsRxView::on_data_rds(const RDSGroupMessage& msg) {
    if (msg.is_debug) {
        if (msg.debug_1 == 777) {
            console.writeln("Partial (A+B): PI=" + to_string_hex(msg.block_a, 4) + " B=" + to_string_hex(msg.block_b, 4));
            return;
        } else if (msg.debug_1 == 888) {
            console.writeln("Partial (A+B+C): PI=" + to_string_hex(msg.block_a, 4) + " C=" + to_string_hex(msg.block_c, 4));
            return;
        }

        // Normál debug statisztika
        uint8_t state = (msg.debug_2 >> 16) & 0xFF;
        uint16_t syndrome = msg.debug_2 & 0xFFFF;
        text_debug.set("B:" + to_string_dec_uint(msg.debug_1) + " S:" + to_string_dec_uint(state) + " SYN:" + to_string_hex(syndrome, 4));
        return;
    }
    // group_count++;

    // 1. PI kód (Állomás azonosító) - Mindig az A blokkban van
    text_pi.set("PI: " + to_string_hex(msg.block_a, 4));

    // A B blokk szerkezete:
    // [15..12] Group Type | [11] Version (A/B) | [10] TP | [9..5] PTY | [4..0] Különböző
    uint8_t group_type = (msg.block_b >> 12) & 0x0F;
    uint8_t group_version = (msg.block_b >> 11) & 0x01;  // 0 = A, 1 = B
    uint8_t pty = (msg.block_b >> 5) & 0x1F;

    text_pty.set("PTY: " + to_string_dec_uint(pty));

    // --- 2. Program Service Name (PS) összerakása ---
    // A PS nevet a 0A és 0B típusú csoportok küldik
    if (group_type == 0) {
        // Az alsó 2 bit mondja meg, hogy a 8 karakterből melyik kettőt kaptuk meg (0-3)
        uint8_t segment = msg.block_b & 0x03;

        // A D blokk tartalmazza a 2 ASCII karaktert
        char c1 = (msg.block_d >> 8) & 0xFF;
        char c2 = msg.block_d & 0xFF;

        // Csak nyomtatható karaktereket fogadunk el (szűrés szemét ellen)
        if (c1 >= 32 && c1 <= 126) ps_name[segment * 2] = c1;
        if (c2 >= 32 && c2 <= 126) ps_name[segment * 2 + 1] = c2;

        text_ps_name.set(ps_name);
    }

    // --- 3. Radio Text (RT) összerakása ---
    // A rádiószöveget a 2A és 2B típusú csoportok küldik
    else if (group_type == 2) {
        // Az alsó 4 bit mondja meg a szegmenst (0-15)
        uint8_t segment = msg.block_b & 0x0F;

        if (group_version == 0) {
            // 2A Csoport: 4 karaktert küld egyszerre (2 a C blokkban, 2 a D blokkban)
            if (!msg.is_c_prime) {
                char c1 = (msg.block_c >> 8) & 0xFF;
                char c2 = msg.block_c & 0xFF;
                char c3 = (msg.block_d >> 8) & 0xFF;
                char c4 = msg.block_d & 0xFF;

                if (c1 >= 32 && c1 <= 126) radio_text[segment * 4] = c1;
                if (c2 >= 32 && c2 <= 126) radio_text[segment * 4 + 1] = c2;
                if (c3 >= 32 && c3 <= 126) radio_text[segment * 4 + 2] = c3;
                if (c4 >= 32 && c4 <= 126) radio_text[segment * 4 + 3] = c4;
            }
        } else {
            // 2B Csoport: 2 karaktert küld (csak a D blokkban)
            char c1 = (msg.block_d >> 8) & 0xFF;
            char c2 = msg.block_d & 0xFF;

            if (c1 >= 32 && c1 <= 126) radio_text[segment * 2] = c1;
            if (c2 >= 32 && c2 <= 126) radio_text[segment * 2 + 1] = c2;
        }

        // Amikor elérjük az utolsó szegmenst, vagy kapunk egy sorvégét (0x0D), kiírjuk a konzolra
        if (segment == 15 || msg.block_d == 0x0D0D || (msg.block_d & 0xFF) == 0x0D) {
            std::string rt_str(radio_text);

            // Trimeljük a felesleges szóközöket a végéről
            rt_str.erase(rt_str.find_last_not_of(" ") + 1);

            if (rt_str.length() > 0) {
                console.writeln("> " + rt_str);
                // Szöveg törlése a következő üzenethez
                memset(radio_text, ' ', 64);
            }
        }
    }
}

RdsRxView::~RdsRxView() {
    receiver_model.disable();
    baseband::shutdown();
}

}  // namespace ui::external_app::rds_rx