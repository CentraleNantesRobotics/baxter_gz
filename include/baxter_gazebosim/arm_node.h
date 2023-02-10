#ifndef BAXTER_GZ_SIM_ARM_H
#define BAXTER_GZ_SIM_ARM_H
#include <rclcpp/node.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace baxter_gazebosim
{

class ArmBridge
{
  using JointCommand = baxter_core_msgs::msg::JointCommand;
  using Range = sensor_msgs::msg::Range;

public: 

  explicit ArmBridge(const std::string &side);
  inline static auto node() {return ros2_node;}

private:
  static std::shared_ptr<rclcpp::Node> ros2_node;
  static ignition::transport::Node gz_node;

  // joint command
  std::map<std::string, ignition::transport::Node::Publisher> pos_pub, vel_pub;
  rclcpp::Subscription<JointCommand>::SharedPtr cmd_sub;
  void republish(const JointCommand &msg);

  // range
  Range range;
  rclcpp::Publisher<Range>::SharedPtr range_pub;
  void republish(const ignition::msgs::LaserScan &scan);

};

}

#endif
