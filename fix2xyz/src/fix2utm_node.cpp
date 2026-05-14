#include "rclcpp/rclcpp.hpp"
#include "fix2utm_node_lib.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LLAUTMTransNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
