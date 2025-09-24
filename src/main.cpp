#include "emcl2/emcl2_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<emcl2::EMcl2Node>();
  rclcpp::Rate loop_rate(node->getOdomFreq());
  while (rclcpp::ok()) {
    node->loop();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
