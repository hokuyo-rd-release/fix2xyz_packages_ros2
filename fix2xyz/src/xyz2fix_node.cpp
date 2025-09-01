#include "rclcpp/rclcpp.hpp"
#include "xyz2fix_node_lib.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<XYZLLATransNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
