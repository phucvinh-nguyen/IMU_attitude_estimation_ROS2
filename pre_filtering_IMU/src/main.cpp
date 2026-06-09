#include "rclcpp/rclcpp.hpp"
#include "pre_filtering_imu/imu_prefilter_node.h"

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  
  // Create Node via Shared Pointer
  auto prefilter_node = std::make_shared<IMUPrefilterNode>();

  rclcpp::spin(prefilter_node);

  rclcpp::shutdown();

  return 0;
}
