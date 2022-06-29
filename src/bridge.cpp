#include <baxter_gazebosim/bridge.h>
#include <rclcpp/rclcpp.hpp>

using namespace baxter_gazebosim;

BaxterBridge::BaxterBridge() : Node("baxter_gz_bridge")
{
  const auto ns{declare_parameter<std::string>("name", "baxter")};

  // init publishers to Gazebo
  for(const std::string joint: {
      "head_pan",
      "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2",
      "left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"})
  {
    const auto gz_topic{"/model/" + ns + "/joint/" + joint + "/0/cmd_"};

    pos_pub[joint] = node.Advertise<ignition::msgs::Double>(gz_topic + "pos");
    vel_pub[joint] = node.Advertise<ignition::msgs::Double>(gz_topic + "vel");

    left_sub = create_subscription<JointCommand>("/robot/limb/left/joint_command", 5,
                                                 [&](const JointCommand &msg){republish(msg);});
    right_sub = create_subscription<JointCommand>("/robot/limb/right/joint_command", 5,
                                                  [&](const JointCommand &msg){republish(msg);});

  }
}

void BaxterBridge::republish(const JointCommand &msg)
{
  static ignition::msgs::Double gz_msg;

  auto &publishers = msg.mode == msg.POSITION_MODE ? pos_pub : vel_pub;

  size_t idx{};
  for(const auto &name: msg.names)
  {
    if(const auto pub{publishers.find(name)}; pub != publishers.end())
    {
      gz_msg.set_data(msg.command[idx]);
      pub->second.Publish(gz_msg);
    }
    idx++;
  }
}

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto robot{std::make_shared<BaxterBridge>()};

  rclcpp::spin(robot);
}
