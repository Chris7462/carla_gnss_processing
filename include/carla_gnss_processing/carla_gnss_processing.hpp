#pragma once

#include <memory>

#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sad_msgs/msg/gnss.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "carla_gnss_processing/map_projection.hpp"


/**
 * Turns the two CARLA GNSS sensors of a vehicle into one sad_msgs/Gnss, the way a dual-antenna
 * RTK receiver reports: the position of the main antenna plus the heading of the baseline from
 * the main to the auxiliary antenna.
 *
 * The two readings are paired by exact time stamp (both sensors must tick on the same simulation
 * step). The heading is valid only if the measured baseline has the expected length.
 */
class CarlaGnssProcessing : public rclcpp::Node
{
public:
  CarlaGnssProcessing();

private:
  using NavSatFix = sensor_msgs::msg::NavSatFix;

  void gnss_callback(
    const NavSatFix::ConstSharedPtr & main_msg, const NavSatFix::ConstSharedPtr & aux_msg);

  std::unique_ptr<MapProjection> projection_;

  double baseline_length_ = 2.0;      // distance between the antennas [m]
  double baseline_tolerance_ = 0.5;   // max deviation from it for a valid heading [m]

  message_filters::Subscriber<NavSatFix> main_sub_;
  message_filters::Subscriber<NavSatFix> aux_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<NavSatFix, NavSatFix>> sync_;

  rclcpp::Publisher<sad_msgs::msg::Gnss>::SharedPtr gnss_pub_;
};
