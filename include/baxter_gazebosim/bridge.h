#ifndef BAXTER_GAZEBOSIM_BRIDGE_H
#define BAXTER_GAZEBOSIM_BRIDGE_H

#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#include <rclcpp/node.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>

namespace baxter_gazebosim
{

class BaxterBridge : public rclcpp::Node
{
  using JointCommand = baxter_core_msgs::msg::JointCommand;

public:
  explicit BaxterBridge();

private:

  void republish(const JointCommand &msg);

  ignition::transport::Node node;
  std::map<std::string, ignition::transport::Node::Publisher> pos_pub, vel_pub;
  rclcpp::Subscription<JointCommand>::SharedPtr left_sub, right_sub;
};
}



#endif
