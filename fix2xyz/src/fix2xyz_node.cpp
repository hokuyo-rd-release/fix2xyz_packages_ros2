#include "rclcpp/rclcpp.hpp"
#include "fix2xyz_node_lib.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LLAXYZTransNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
