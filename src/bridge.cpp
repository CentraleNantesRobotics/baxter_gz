#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <baxter_gz/arm_node.h>

#ifdef FOR_IGNITION_FORTRESS
#include <ignition/msgs/model.pb.h>
#else
#include <gz/msgs/model.pb.h>
#endif

using baxter_gz::Arm;
using namespace std::chrono_literals;

class Bridge : public rclcpp::Node
{
  using Model = gz::msgs::Model;

public:

  explicit inline Bridge()
    : rclcpp::Node("baxter_gz_bridge"),
      left("left", this, gz_node),
      right("right", this, gz_node)
  {

    const auto world{declare_parameter("world", "baxter_world")};
    const auto state_topic{"/world/" + world + "/model/baxter/joint_state"};

    std::function<void(const Model&)> sub_cb{[&](const auto &msg){state_cb(msg);}};
    gz_node.Subscribe(state_topic, sub_cb);

    timer = create_wall_timer(50ms, [&]()
    {left.move(); right.move();});
  }

private:
  gz::transport::Node gz_node;

  rclcpp::TimerBase::SharedPtr timer;

  Arm left, right;

  inline void state_cb(const Model &model)
  {
    for(auto i = 0; i < model.joint_size(); ++i)
    {
      const auto &joint{model.joint(i)};
      Arm::state[joint.name()] = joint.axis1().position();
    }
  }
};


int main (int argc, char** argv)
{    
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bridge>());
}
