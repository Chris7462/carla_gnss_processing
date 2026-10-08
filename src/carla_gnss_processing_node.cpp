#include "carla_gnss_processing/carla_gnss_processing.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarlaGnssProcessing>());
  rclcpp::shutdown();

  return 0;
}
