#pragma once

#include <Eigen/Core>
#include <GeographicLib/TransverseMercator.hpp>

/**
 * Moves the geodetic coordinates reported by CARLA to another place on Earth.
 *
 * CARLA turns map positions (metres) into latitude / longitude with the projection in the map's
 * OpenDRIVE geoReference, a transverse Mercator that the built-in towns centre on latitude 0,
 * longitude 0. That point is both the equator and the border between UTM zones 30 and 31, so a
 * consumer that converts the readings to UTM sees its coordinates jump by hundreds or thousands
 * of kilometres whenever the vehicle crosses either line.
 *
 * ToMap() undoes CARLA's projection and returns the position in map metres (east, north).
 * ToGeodetic() places those metres on the UTM grid around a reference point far from any zone
 * border and returns the latitude / longitude there. Converting that result to UTM gives the map
 * metres back exactly (plus a constant), with no jump.
 */
class MapProjection
{
public:
  struct Options
  {
    /// Projection of the CARLA map: +proj=tmerc parameters of its geoReference
    double map_lat_0_ = 0.0;                    // [deg]
    double map_lon_0_ = 0.0;                    // [deg]
    double map_k_ = 1.0;                        // scale factor
    double map_ellps_a_ = 6378137.0;            // semi-major axis [m] (WGS84)
    double map_ellps_rf_ = 298.257223563;       // inverse flattening (WGS84); 0 for a sphere

    /// Where the map origin is placed. Any point away from the equator works; a longitude on a
    /// UTM central meridian (..., 3, 9, 15, ... deg) keeps the whole map inside one zone.
    double origin_lat_ = 45.0;                  // [deg]
    double origin_lon_ = 9.0;                   // [deg]
  };

  explicit MapProjection(const Options & options);

  /// CARLA latitude / longitude [deg] -> metres east / north of the map origin
  Eigen::Vector2d ToMap(double lat, double lon) const;

  /// Metres east / north of the map origin -> latitude / longitude [deg] near the reference point
  void ToGeodetic(const Eigen::Vector2d & east_north, double & lat, double & lon) const;

private:
  Options options_;
  GeographicLib::TransverseMercator map_tm_;
  double map_north_0_ = 0.0;   // northing of map_lat_0_ in the map projection [m]

  // UTM coordinates of the reference point
  int origin_zone_ = 0;
  bool origin_northp_ = true;
  double origin_east_ = 0.0;    // [m]
  double origin_north_ = 0.0;   // [m]
};
