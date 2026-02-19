#ifndef MID360_GZ_PLUGIN__CSV_READER_HPP_
#define MID360_GZ_PLUGIN__CSV_READER_HPP_

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace mid360_gz_plugin
{

/// One ray direction: azimuth (rad), zenith (rad). Zenith is converted from
/// CSV (deg) to standard right-hand: zenith_rad = (zenith_deg * deg2rad) - M_PI_2.
using RayDirection = std::pair<double, double>;

/// Reads Livox-style CSV: header "Time/s,Azimuth/deg,Zenith/deg", then rows of 3 doubles.
/// Returns ray directions as (azimuth_rad, zenith_rad). Skips empty lines and invalid rows.
inline bool read_scan_csv(const std::string & file_path, std::vector<RayDirection> & out_directions)
{
  const double deg2rad = M_PI / 180.0;
  std::ifstream f(file_path);
  if (!f.is_open()) {
    return false;
  }
  std::string header;
  if (!std::getline(f, header)) {
    return false;
  }
  out_directions.clear();
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) { continue; }
    std::istringstream ss(line);
    std::string time_s, azimuth_s, zenith_s;
    if (!std::getline(ss, time_s, ',') ||
        !std::getline(ss, azimuth_s, ',') ||
        !std::getline(ss, zenith_s, ','))
    {
      continue;
    }
    try {
      double azimuth_deg = std::stod(azimuth_s);
      double zenith_deg = std::stod(zenith_s);
      double azimuth_rad = azimuth_deg * deg2rad;
      double zenith_rad = zenith_deg * deg2rad - M_PI_2;  // right-hand convention
      out_directions.emplace_back(azimuth_rad, zenith_rad);
    } catch (...) {
      continue;
    }
  }
  return !out_directions.empty();
}

}  // namespace mid360_gz_plugin

#endif  // MID360_GZ_PLUGIN__CSV_READER_HPP_
