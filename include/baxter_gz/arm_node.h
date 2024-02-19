#ifndef BAXTER_GZ_ARM_H
#define BAXTER_GZ_ARM_H

#include <rclcpp/node.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/range.hpp>

#ifdef IGNITION_GAZEBO
#include <ignition/transport/Node.hh>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/double.pb.h>
#define GZ_NS ignition
#else
#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/double.pb.h>
#define GZ_NS gz
#endif

namespace baxter_gz
{

class Arm
{
  using JointCommand = baxter_core_msgs::msg::JointCommand;
  using Range = sensor_msgs::msg::Range;

public: 

  explicit Arm(const std::string &side, rclcpp::Node *ros, GZ_NS::transport::Node &gz);
  static std::unordered_map<std::string, double> state;

  void move();

private: 

  static rclcpp::Node* node;

  // joint command
  JointCommand last_cmd;
  std::unordered_map<std::string, GZ_NS::transport::Node::Publisher> pos_pub, vel_pub;
  rclcpp::Subscription<JointCommand>::SharedPtr cmd_sub;
  void republish(const JointCommand &msg);


  // range
  Range range;
  rclcpp::Publisher<Range>::SharedPtr range_pub;
  void republish(const GZ_NS::msgs::LaserScan &scan);
};

}

#endif
