#ifndef BAXTER_GZ_SIM_ARM_H
#define BAXTER_GZ_SIM_ARM_H

#include <rclcpp/node.hpp>
#include <ignition/transport/Node.hh>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace baxter_gazebosim
{

class Arm
{
  using JointCommand = baxter_core_msgs::msg::JointCommand;
  using Range = sensor_msgs::msg::Range;

public: 

  explicit Arm(const std::string &side, rclcpp::Node *ros, ignition::transport::Node &gz);   
  static std::unordered_map<std::string, double> state;

  void move();

private: 

  static rclcpp::Node* node;

  // joint command
  JointCommand last_cmd;
  std::unordered_map<std::string, ignition::transport::Node::Publisher> pos_pub, vel_pub;
  rclcpp::Subscription<JointCommand>::SharedPtr cmd_sub;
  void republish(const JointCommand &msg);


  // range
  Range range;
  rclcpp::Publisher<Range>::SharedPtr range_pub;
  void republish(const ignition::msgs::LaserScan &scan);
};

}

#endif
