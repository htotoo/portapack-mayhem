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

#include "ui_satpass.hpp"

#include "rtc_time.hpp"
#include "string_format.hpp"
#include "file_path.hpp"
#include "portapack_persistent_memory.hpp"
#include "file_reader.hpp"

using namespace portapack;
using namespace ui;
using FileLineReader = BufferLineReader<File>;

namespace ui::external_app::satpass {

void SatpassView::focus() {
    // geopos.focus();
}

SatpassView::SatpassView(NavigationView& nav)
    : nav_{nav} {
    add_children({&console});
    load_tle_from_sat(satName, tle_data);
}

SatpassView::~SatpassView() {
}

void SatpassView::load_tle_from_sat(std::string sat_name, TLEData& tle_data) {
    // tle_file
    File f;
    auto error = f.open(tle_file);
    console.writeln("Opening file");
    if (error) {
        console.writeln("Error opening file");
        return;
    }
    console.writeln("File opened");
    std::string line;
    std::string sat_line1, sat_line2;
    uint8_t found = 0;

    auto reader = FileLineReader(f);

    for (auto& line : reader) {
        if (line.find(sat_name) != std::string::npos) {
            found = 1;
            continue;
        }
        if (found == 1) {
            sat_line1 = line;
            found = 2;
            continue;
        }
        if (found == 2) {
            sat_line2 = line;
            break;
        }
    }
    if (!found) {
        // Handle satellite not found error
        console.writeln("Satellite not found");
        return;
    }
    tle_data.parse_tle(sat_line1, sat_line2, satName);
    console.writeln("TLE data parsed");
    update();
}

double SatpassView::julianDate(int year, int month, int day, int hour, int minute, int second) {
    // Adjust for the fact that January and February are considered
    // months 13 and 14 of the previous year
    if (month <= 2) {
        year -= 1;
        month += 12;
    }

    // Calculate the Julian Day Number
    int A = year / 100;
    int B = 2 - A + (A / 4);

    double JDN = floor(365.25 * (year + 4716)) +
                 floor(30.6001 * (month + 1)) +
                 day + B - 1524.5;

    // Add the fraction of a day
    double fracDay = (hour + minute / 60.0 + second / 3600.0) / 24.0;

    // Return the Julian Date
    return JDN + fracDay;
}

void SatpassView::calculateSatellitePosition(const TLEData& tle, double julian_date, double& x, double& y, double& z) {
    if (!tle.is_ok) {
        x = y = z = 0;
        return;
    }

    // Convert mean motion to radians per minute
    double n0 = tle.mean_motion * TWO_PI / MINUTES_PER_DAY;

    // Time since epoch in minutes
    double t = (julian_date - tle.epoch) * MINUTES_PER_DAY;

    // Convert inclination and other angles to radians
    double incl = tle.inclination * PI / 180.0;
    double raan = tle.right_ascension * PI / 180.0;
    double arg_perigee = tle.argument_of_perigee * PI / 180.0;
    double mean_anomaly = tle.mean_anomaly * PI / 180.0;

    // Calculate auxiliary values
    double a1 = pow(XKE / n0, 2.0 / 3.0);
    double cosio = cos(incl);
    double sinio = sin(incl);
    double eta = 1.0 + (cosio > 0 ? 1 : -1) * sqrt(1.0 - tle.eccentricity * tle.eccentricity);
    double C1 = tle.bstar * 2.0 * CK2 * a1 * n0 * pow(EARTH_RADIUS_KM / (1.0 - tle.eccentricity * tle.eccentricity), 3.5) / sqrt(1.0 - tle.eccentricity * tle.eccentricity);
    double a0 = a1 * (1.0 - C1 * t / 3.0 - C1 * C1 * t * t / 2.0);
    double delta1 = 1.5 * CK2 * (3.0 * cosio * cosio - 1.0) / pow(1.0 - tle.eccentricity * tle.eccentricity, 1.5);
    double a = a0 / (1.0 - C1 * t);
    double e = tle.eccentricity - tle.bstar * CK2 * t * eta / (a * (1.0 - tle.eccentricity * tle.eccentricity));

    // Update for secular gravity and atmospheric drag
    double n = n0 + (3.0 * n0 * delta1 * t + C1 * t * t * (1.0 + 3.0 * eta * t / 2.0)) / (a * a);

    // Kepler's equation
    double M = mean_anomaly + n * t;
    M = fmod(M, TWO_PI);
    if (M < 0) M += TWO_PI;

    // Solve Kepler's equation using Newton's method
    double E = M;
    for (int i = 0; i < 10; i++) {
        double E_next = E - (E - e * sin(E) - M) / (1 - e * cos(E));
        if (fabs(E_next - E) < 1e-8) break;
        E = E_next;
    }

    // Calculate true anomaly
    double nu = 2 * atan2(sqrt(1 + e) * sin(E / 2), sqrt(1 - e) * cos(E / 2));

    // Calculate position in orbital plane
    double r = a * (1 - e * cos(E));
    double x_orbit = r * cos(nu);
    double y_orbit = r * sin(nu);

    // Rotate to ECI frame
    double cosRAAN = cos(raan);
    double sinRAAN = sin(raan);
    double cosAOP = cos(arg_perigee);
    double sinAOP = sin(arg_perigee);

    x = (cosRAAN * cosAOP - sinRAAN * sinAOP * cosio) * x_orbit + (-cosRAAN * sinAOP - sinRAAN * cosAOP * cosio) * y_orbit;
    y = (sinRAAN * cosAOP + cosRAAN * sinAOP * cosio) * x_orbit + (-sinRAAN * sinAOP + cosRAAN * cosAOP * cosio) * y_orbit;
    z = (sinAOP * sinio) * x_orbit + (cosAOP * sinio) * y_orbit;
}

double SatpassView::calculateGMST(double julianDate) {
    double T = (julianDate - 2451545.0) / 36525.0;
    double gmst = 280.46061837 + 360.98564736629 * (julianDate - 2451545.0) +
                  0.000387933 * T * T - T * T * T / 38710000.0;
    gmst = fmod(gmst, 360.0);
    if (gmst < 0) gmst += 360.0;
    return gmst * DEG_TO_RAD;
}

void SatpassView::calculateObserverPosition(double lat_rad, double lon_rad, double& observer_x, double& observer_y, double& observer_z) {
    double f = EARTH_FLATTENING;
    double e2 = 2 * f - f * f;
    double cos_lat = cos(lat_rad);
    double sin_lat = sin(lat_rad);
    double N = EARTH_RADIUS_KM / sqrt(1 - e2 * sin_lat * sin_lat);
    double alt_km = 0.1;
    observer_x = (N + alt_km) * cos_lat * cos(lon_rad);
    observer_y = (N + alt_km) * cos_lat * sin(lon_rad);
    observer_z = ((1 - e2) * N + alt_km) * sin_lat;
}

void SatpassView::ecefToEci(double ecef_x, double ecef_y, double ecef_z, double gmst, double& eci_x, double& eci_y, double& eci_z) {
    // Precompute sine and cosine of GMST for efficiency
    double cos_gmst = cos(gmst);
    double sin_gmst = sin(gmst);
    eci_x = ecef_x * cos_gmst - ecef_y * sin_gmst;
    eci_y = ecef_x * sin_gmst + ecef_y * cos_gmst;
    eci_z = ecef_z;
}

void SatpassView::calculateTopocentricCoordinates(double obs_x, double obs_y, double obs_z, double sat_x, double sat_y, double sat_z, double& top_s, double& top_e, double& top_z) {
    // Convert observer's latitude and longitude to radians
    double dx = sat_x - obs_x;
    double dy = sat_y - obs_y;
    double dz = sat_z - obs_z;

    double lat = atan2(obs_z, sqrt(obs_x * obs_x + obs_y * obs_y));
    double lon = atan2(obs_y, obs_x);

    double sin_lat = sin(lat);
    double cos_lat = cos(lat);
    double sin_lon = sin(lon);
    double cos_lon = cos(lon);

    top_s = sin_lat * cos_lon * dx + sin_lat * sin_lon * dy - cos_lat * dz;
    top_e = -sin_lon * dx + cos_lon * dy;
    top_z = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
}

void SatpassView::update() {
    // Check if TLE data is valid
    if (!tle_data.is_ok) {
        load_tle_from_sat(satName, tle_data);
    }
    if (!tle_data.is_ok) {
        console.writeln("TLE data not found");
        return;
    }

    // Display selected satellite information
    console.writeln("Sel sat: " + tle_data.name);
    console.writeln("SatID: " + to_string_decimal(tle_data.satellite_number, 0));
    console.writeln("Curr loc: " + to_string_decimal(lat, 6) + ", " + to_string_decimal(lon, 6));

    // Get current time
    auto current_time = rtc_time::now();

    // Constants for calculations
    const double pi = 3.14159265358979323846;
    const double deg2rad = pi / 180.0;
    const double rad2deg = 180.0 / pi;

    // Convert latitude and longitude to radians
    double lat_rad = lat * deg2rad;
    double lon_rad = lon * deg2rad;

    // Calculate Julian date
    int year = current_time.year();
    int month = current_time.month();
    int day = current_time.day();
    int hour = current_time.hour();
    int minute = current_time.minute();
    int second = current_time.second();

    double jd = julianDate(year, month, day, hour, minute, second);
    console.writeln("JD: " + to_string_decimal(jd, 6));

    // Calculate satellite position
    double sat_x, sat_y, sat_z;
    calculateSatellitePosition(tle_data, jd, sat_x, sat_y, sat_z);

    // Calculate GMST
    double gmst = calculateGMST(jd);

    // Calculate observer's position in ECEF coordinates
    double observer_x, observer_y, observer_z;
    calculateObserverPosition(lat_rad, lon_rad, observer_x, observer_y, observer_z);

    // Convert observer's position to ECI coordinates
    double observer_x_eci, observer_y_eci, observer_z_eci;
    ecefToEci(observer_x, observer_y, observer_z, gmst, observer_x_eci, observer_y_eci, observer_z_eci);

    // Calculate range vector from observer to satellite
    double range_x = sat_x - observer_x_eci;
    double range_y = sat_y - observer_y_eci;
    double range_z = sat_z - observer_z_eci;
    double sat_range = sqrt(range_x * range_x + range_y * range_y + range_z * range_z);

    // Calculate topocentric coordinates
    double top_s, top_e, top_z;
    calculateTopocentricCoordinates(range_x, range_y, range_z, lat_rad, lon_rad, gmst, top_s, top_e, top_z);

    // Calculate satellite elevation and azimuth
    double sat_elevation = asin(top_z / sat_range) * rad2deg;
    double sat_azimuth = atan2(-top_e, top_s) * rad2deg;
    if (sat_azimuth < 0) {
        sat_azimuth += 360.0;
    }

    // Check if satellite is visible and update console
    if (sat_elevation > 0) {
        console.writeln("Satellite is visible");
        console.writeln("Elevation: " + to_string_decimal(sat_elevation, 2) + "°");
        console.writeln("Azimuth: " + to_string_decimal(sat_azimuth, 2) + "°");
    } else {
        console.writeln("Satellite is NOT visible");
        console.writeln("Elevation: " + to_string_decimal(sat_elevation, 2) + "°");
    }
}

void SatpassView::on_gps(const GPSPosDataMessage* msg) {
    lat = msg->lat;
    lon = msg->lon;
    update();
}

}  // namespace ui::external_app::satpass