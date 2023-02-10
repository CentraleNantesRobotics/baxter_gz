#include <rclcpp/rclcpp.hpp>
#include <baxter_gazebosim/arm_node.h>

using baxter_gazebosim::ArmBridge;

int main (int argc, char** argv)
{    
  rclcpp::init(argc, argv);

  [[maybe_unused]] auto left{ArmBridge("left")};
  [[maybe_unused]] auto right{ArmBridge("right")};

  rclcpp::spin(ArmBridge::node());

}
