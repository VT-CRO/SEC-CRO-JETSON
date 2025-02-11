#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

using Twist = geometry_msgs::msg::TwistStamped;

namespace BT {
  template <> inline Twist convertFromString(StringView str) {
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            auto twist = Twist();


            twist.twist.linear.x = convertFromString<double>(parts[0]);
            twist.twist.linear.y = convertFromString<double>(parts[1]);

            twist.twist.angular.z = convertFromString<double>(parts[2]);

            return twist;
        }
  }
}


// We want to use this custom type
class SetChassisVelocity : public BT::SyncActionNode
{
public:
  SetChassisVelocity(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts() {
    return { BT::InputPort<Twist>("target"), BT::InputPort<int>("duration_s") };
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;
private:
  rclcpp::Publisher<Twist>::SharedPtr publisher_;

  float val;

  rclcpp::Node::SharedPtr node_ptr_;

};