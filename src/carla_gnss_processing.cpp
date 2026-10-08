#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "carla_gnss_processing/carla_gnss_processing.hpp"


CarlaGnssProcessing::CarlaGnssProcessing()
: Node("carla_gnss_processing_node")
{
  const std::string main_topic =
    declare_parameter<std::string>("main_topic", "/carla/hero/gnss_main");
  const std::string aux_topic =
    declare_parameter<std::string>("aux_topic", "/carla/hero/gnss_aux");
  const std::string gnss_topic = declare_parameter<std::string>("gnss_topic", "/carla/hero/gnss");

  baseline_length_ = declare_parameter<double>("baseline_length", baseline_length_);
  baseline_tolerance_ = declare_parameter<double>("baseline_tolerance", baseline_tolerance_);

  map_frame_ = declare_parameter<std::string>("map_frame", map_frame_);
  gnss_frame_ = declare_parameter<std::string>("gnss_frame", gnss_frame_);

  MapProjection::Options options;
  options.map_lat_0_ = declare_parameter<double>("map_lat_0", options.map_lat_0_);
  options.map_lon_0_ = declare_parameter<double>("map_lon_0", options.map_lon_0_);
  options.map_k_ = declare_parameter<double>("map_k", options.map_k_);
  options.map_ellps_a_ = declare_parameter<double>("map_ellps_a", options.map_ellps_a_);
  options.map_ellps_rf_ = declare_parameter<double>("map_ellps_rf", options.map_ellps_rf_);
  options.origin_lat_ = declare_parameter<double>("origin_lat", options.origin_lat_);
  options.origin_lon_ = declare_parameter<double>("origin_lon", options.origin_lon_);
  projection_ = std::make_unique<MapProjection>(options);

  // Reliable, matching the CARLA publishers and the SAD subscribers
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

  gnss_pub_ = create_publisher<sad_msgs::msg::Gnss>(gnss_topic, qos);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  main_sub_.subscribe(this, main_topic, qos);
  aux_sub_.subscribe(this, aux_topic, qos);

  // Exact time: a pair one simulation step apart would turn the vehicle motion into heading error
  sync_ = std::make_shared<message_filters::TimeSynchronizer<NavSatFix, NavSatFix>>(
    10, main_sub_, aux_sub_);
  sync_->registerCallback(
    std::bind(&CarlaGnssProcessing::gnss_callback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(), "Pairing %s and %s into %s (baseline %.2f +/- %.2f m, map origin at %.4f, %.4f)",
    main_topic.c_str(), aux_topic.c_str(), gnss_topic.c_str(),
    baseline_length_, baseline_tolerance_, options.origin_lat_, options.origin_lon_);
}

void CarlaGnssProcessing::gnss_callback(
  const NavSatFix::ConstSharedPtr & main_msg, const NavSatFix::ConstSharedPtr & aux_msg)
{
  // Both antennas in map metres (east, north)
  const Eigen::Vector2d main_pos = projection_->ToMap(main_msg->latitude, main_msg->longitude);
  const Eigen::Vector2d aux_pos = projection_->ToMap(aux_msg->latitude, aux_msg->longitude);

  // Baseline from the main to the auxiliary antenna
  const Eigen::Vector2d baseline = aux_pos - main_pos;
  const double length = baseline.norm();

  // Compass heading of the baseline: clockwise from north, in [0, 360)
  double heading = std::atan2(baseline.x(), baseline.y()) * 180.0 / M_PI;
  if (heading < 0.0) {
    heading += 360.0;
  }

  sad_msgs::msg::Gnss msg;
  msg.header = main_msg->header;
  msg.status = sad_msgs::msg::Gnss::STATUS_FIXED;
  projection_->ToGeodetic(main_pos, msg.latitude, msg.longitude);
  msg.altitude = main_msg->altitude;
  msg.heading = heading;
  msg.heading_valid = std::abs(length - baseline_length_) <= baseline_tolerance_;

  if (!msg.heading_valid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Baseline is %.3f m, expected %.2f +/- %.2f m: heading marked invalid",
      length, baseline_length_, baseline_tolerance_);
  }

  gnss_pub_->publish(msg);

  // tf map -> gnss_link: main antenna in CARLA map coordinates, x east, y north, z altitude.
  // The rotation is the yaw of the baseline (counter-clockwise from east); identity while the
  // heading is invalid.
  const Eigen::Vector3d trans(main_pos.x(), main_pos.y(), main_msg->altitude);
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
  if (msg.heading_valid) {
    const double yaw = std::atan2(baseline.y(), baseline.x());
    quat = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = main_msg->header.stamp;  // data time, not now()
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = gnss_frame_;
  tf_msg.transform.translation = tf2::toMsg2(trans);
  tf_msg.transform.rotation = tf2::toMsg(quat);
  tf_broadcaster_->sendTransform(tf_msg);
}
