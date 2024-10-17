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

#ifndef __UI_SATPASS_H__
#define __UI_SATPASS_H__

#include "ui.hpp"
#include "ui_language.hpp"
#include "ui_navigation.hpp"
#include "ui_geomap.hpp"
#include "app_settings.hpp"
#include "utility.hpp"

using namespace ui;

namespace ui::external_app::satpass {

struct TLEData {
    std::string name{};
    int satellite_number{};
    double inclination{};
    double right_ascension{};
    double eccentricity{};
    double argument_of_perigee{};
    double mean_anomaly{};
    double mean_motion{};
    double epoch{};  // Julian Date
    double bstar{};  // B* drag term
    bool is_ok = false;

    TLEData() = default;

    TLEData(const std::string& name, const std::string& line1, const std::string& line2)
        : name(name) {
        parse_tle(line1, line2);
    }

    void parse_tle(const std::string& line1, const std::string& line2, const std::string& name = "") {
        if (line1.size() < 69 || line2.size() < 69) {
            is_ok = false;
            return;
        }
        if (!name.empty()) {
            this->name = name;
        }

        // Parse line1
        satellite_number = std::stoi(line1.substr(2, 5));

        // Parse epoch (year and day of year)
        int year = std::stoi(line1.substr(18, 2));
        double day_of_year = std::stod(line1.substr(20, 12));
        year += (year < 57) ? 2000 : 1900;  // Adjust for year 2000+
        epoch = julian_date(year, day_of_year);

        // Parse B* drag term
        int bstar_exponent = std::stoi(line1.substr(59, 2));
        bstar = std::stod(line1.substr(53, 6)) * std::pow(10, bstar_exponent);

        // Parse line2
        inclination = std::stod(line2.substr(8, 8));
        right_ascension = std::stod(line2.substr(17, 8));
        eccentricity = std::stod("0." + line2.substr(26, 7));
        argument_of_perigee = std::stod(line2.substr(34, 8));
        mean_anomaly = std::stod(line2.substr(43, 8));
        mean_motion = std::stod(line2.substr(52, 11));

        is_ok = true;
    }
    double julian_date(int year, double day_of_year) {
        int a = (14 - 1) / 12;
        int y = year + 4800 - a;
        int m = 1 + 12 * a - 3;

        double jd = day_of_year + 365 * y + y / 4 - y / 100 + y / 400 - 32045;
        return jd;
    }
};

class SatpassView : public View {
   public:
    SatpassView(NavigationView& nav);
    ~SatpassView();

    void focus() override;

    std::string title() const override {
        return "SatpPass";
    };

   private:
    std::filesystem::path tle_file = u"satpass/data.tle";
    NavigationView& nav_;
    Console console{{0, 0, 320, 240}};

    TLEData tle_data{};
    float lat = 0;
    float lon = 0;
    void load_tle_from_sat(std::string sat_name, TLEData& tle_data);
    void on_gps(const GPSPosDataMessage* msg);
    void update();
    double julianDate(int year, int month, int day, int hour, int minute, int second);
    void calculateSatellitePosition(const TLEData& tle, double julian_date, double& x, double& y, double& z);
    double calculateGMST(double julianDate);
    void calculateObserverPosition(double lat_rad, double lon_rad, double& observer_x, double& observer_y, double& observer_z);
    void ecefToEci(double ecef_x, double ecef_y, double ecef_z, double gmst, double& eci_x, double& eci_y, double& eci_z);
    void calculateTopocentricCoordinates(double obs_lat, double obs_lon, double obs_alt, double sat_x, double sat_y, double sat_z, double& top_s, double& top_e, double& top_z);
    std::string satName = "NOAA 15";

    // Constants
    double PI = 3.141592653589793;
    double TWO_PI = 2.0 * PI;
    double MINUTES_PER_DAY = 1440.0;
    double EARTH_RADIUS_KM = 6378.137;
    double J2 = 0.00108262998905;
    double XKE = 7.43669161E-2;
    double CK2 = 5.413080E-4;
    double DEG_TO_RAD = PI / 180.0;
    double RAD_TO_DEG = 180.0 / PI;
    double EARTH_FLATTENING = 1.0 / 298.257223563;  // Earth's flattening factor
    double OMEGA_EARTH = 7.2921150e-5;              // Earth's rotation rate in rad/s

    MessageHandlerRegistration message_handler_gps{
        Message::ID::GPSPosData,
        [this](Message* const p) {
            const auto message = static_cast<const GPSPosDataMessage*>(p);
            this->on_gps(message);
        }};
};
};  // namespace ui::external_app::satpass

#endif /*__UI_satpass_H__*/
