#include <baxter_gz/arm_node.h>

namespace baxter_gz
{

std::unordered_map<std::string, double> Arm::state;
rclcpp::Node* Arm::node;

Arm::Arm(const std::string &side, rclcpp::Node *ros, gz::transport::Node &gz)
{
  node = ros;
  const auto name{node->get_effective_namespace().substr(1)};

  // init publishers to Gazebo
  for(const auto suffix: {"s0", "s1", "e0", "e1", "w0", "w1", "w2"})
  {
    const auto joint{side + "_" + suffix};
    const auto gz_topic_pos{"/model/" + name + "/joint/" + joint + "/0/cmd_pos"};
    const auto gz_topic_vel{joint + "_cmd_vel"};

    vel_pub[joint] = gz.Advertise<gz::msgs::Double>(gz_topic_vel);

    cmd_sub = ros->create_subscription<JointCommand>("limb/" + side + "/joint_command", 5,
                                                     [&](const JointCommand &msg)
    {last_cmd = msg;}
    );
  }

  // init range
  range.radiation_type = range.INFRARED;
  range.header.frame_id = side + "_hand_range";
  range_pub = ros->create_publisher<Range>("range/" + side + "_hand_range/state", 1);

  std::function<void(const gz::msgs::LaserScan&)> sub_cb{[&](const auto &msg){republish(msg);}};
  gz.Subscribe("/" + side + "_arm/range", sub_cb);
}

void Arm::republish(const JointCommand & msg)
{
  static gz::msgs::Double gz_cmd;
  auto &publishers = msg.mode == msg.POSITION_MODE ? pos_pub : vel_pub;

  auto joint_cmd{msg.command.begin()};
  for(const auto &joint: msg.names)
  {
    if(const auto pub{publishers.find(joint)}; pub != publishers.end())
    {
      gz_cmd.set_data(*joint_cmd);
      pub->second.Publish(gz_cmd);
    }
    joint_cmd++;
  }
}

void Arm::move()
{
  if(last_cmd.names.empty())
    return;

  const auto use_position{last_cmd.mode == last_cmd.POSITION_MODE};
  static gz::msgs::Double gz_cmd;

  auto joint_cmd{last_cmd.command.begin()};
  for(const auto &joint: last_cmd.names)
  {
    if(const auto pub{vel_pub.find(joint)}; pub != vel_pub.end())
    {
      if(use_position)
      {
        gz_cmd.set_data(10.*(*joint_cmd - state[joint]));
      }
      else
      {
        gz_cmd.set_data(*joint_cmd);
      }
      pub->second.Publish(gz_cmd);
    }
    joint_cmd++;
  }
}

void Arm::republish(const gz::msgs::LaserScan &scan)
{
  range.max_range = scan.range_max();
  range.min_range = scan.range_min();
  range.field_of_view = scan.angle_max();
  range.range = range.max_range+1;
  for(auto i = 0; i < scan.ranges_size(); ++i)
  {
    if(scan.ranges(i) <= scan.range_max() && scan.ranges(i) >= scan.range_min())
      range.range = std::min<float>(range.range, scan.ranges(i));
  }

  range.header.stamp = node->get_clock()->now();
  range_pub->publish(range);
}

}
