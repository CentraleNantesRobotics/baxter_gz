#include <rclcpp/rclcpp.hpp>
#include <baxter_gazebosim/arm_node.h>

using baxter_gazebosim::ArmBridge;

int main (int argc, char** argv)
{    
  rclcpp::init(argc, argv);

//  rclcpp::executors::SingleThreadedExecutor exec;
//  exec.add_node(std::make_shared<ArmBridge>("left"));
  //exec.add_node(std::make_shared<ArmBridge>("right"));

  rclcpp::spin(std::make_shared<ArmBridge>("left"));

  //exec.spin();
}
