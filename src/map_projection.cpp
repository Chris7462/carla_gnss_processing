#include <GeographicLib/UTMUPS.hpp>

#include "carla_gnss_processing/map_projection.hpp"


MapProjection::MapProjection(const Options & options)
: options_(options),
  map_tm_(
    options.map_ellps_a_,
    options.map_ellps_rf_ > 0.0 ? 1.0 / options.map_ellps_rf_ : 0.0,
    options.map_k_)
{
  double east_0 = 0.0;
  map_tm_.Forward(
    options_.map_lon_0_, options_.map_lat_0_, options_.map_lon_0_, east_0, map_north_0_);

  // Throws GeographicLib::GeographicErr if the reference point is not a valid position
  GeographicLib::UTMUPS::Forward(
    options_.origin_lat_, options_.origin_lon_,
    origin_zone_, origin_northp_, origin_east_, origin_north_);
}

Eigen::Vector2d MapProjection::ToMap(double lat, double lon) const
{
  double east = 0.0;
  double north = 0.0;
  map_tm_.Forward(options_.map_lon_0_, lat, lon, east, north);
  return Eigen::Vector2d(east, north - map_north_0_);
}

void MapProjection::ToGeodetic(const Eigen::Vector2d & east_north, double & lat, double & lon) const
{
  GeographicLib::UTMUPS::Reverse(
    origin_zone_, origin_northp_,
    origin_east_ + east_north.x(), origin_north_ + east_north.y(), lat, lon);
}
