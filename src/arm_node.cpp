#include <baxter_gazebosim/arm_node.h>

namespace baxter_gazebosim
{

ArmBridge::ArmBridge(const std::string &side) : Node(side + "_arm_bridge")
{
  // init publishers to Gazebo
  for(const auto suffix: {"s0", "s1", "e0", "e1", "w0", "w1", "w2"})
  {
    const auto joint{side + "_" + suffix};
    const auto gz_topic_pos{"/model/baxter/joint/" + joint + "/0/cmd_pos"};
    const auto gz_topic_vel{joint + "_cmd_vel"};

    pos_pub[suffix] = node.Advertise<ignition::msgs::Double>(gz_topic_pos);
    vel_pub[suffix] = node.Advertise<ignition::msgs::Double>(gz_topic_vel);

    cmd_sub = create_subscription<JointCommand>("/robot/limb/" + side + "/joint_command", 5,
                                                 [&](const JointCommand &msg){republish(msg);});;

  }

  // init range
  range.radiation_type = range.INFRARED;
  range.header.frame_id = side + "_hand_range";
  range_pub = create_publisher<Range>("/robot/range/" + side + "_hand_range/state", 1);

  //std::function<void(const ignition::msgs::LaserScan&)> sub_cb{[&](const auto &msg){republish(msg);}};
  //node.Subscribe("/" + side + "_range", sub_cb);
}

void ArmBridge::republish(const JointCommand &msg)
{
  static ignition::msgs::Double gz_cmd;

  auto &publishers = msg.mode == msg.POSITION_MODE ? pos_pub : vel_pub;

  size_t idx{};
  for(const auto &name: msg.names)
  {
    if(const auto pub{publishers.find(name)}; pub != publishers.end())
    {
      gz_cmd.set_data(msg.command[idx]);
      pub->second.Publish(gz_cmd);
    }
    idx++;
  }
}

void ArmBridge::republish(const ignition::msgs::LaserScan &scan)
{
  auto valid{0};
  range.range = 0;
  for(auto i = 0; i < scan.ranges_size(); ++i)
  {
    if(scan.ranges(i) <= scan.range_max() && scan.ranges(i) >= scan.range_min())
    {
      valid++;
      range.range += scan.ranges(i);
    }
  }

  if(valid)
    range.range /= valid;
  else
    range.range = scan.range_max() + 1.;

  range.header.stamp = get_clock()->now();
  range_pub->publish(range);
}

}
