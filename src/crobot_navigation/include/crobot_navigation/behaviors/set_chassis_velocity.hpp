#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/Twist.msg"
#include "geometry_msgs/Vector3.msg"

using Twist = geometry_msgs::msg::Twist;

namespace BT {
  template <> inline Twist convertFromString(StringView str) {
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            auto linear = Twist::Vector3::linear();
            auto angular = Twist::Vector3::angular();

            linear.x = convertFromString<double>(parts[0]);
            linear.y = convertFromString<double>(parts[1]);

            angular.z = convertFromString<double>(parts[2]);

            return ;
        }
  }
}


// We want to use this custom type
class SetChassisVelocity : public BT::SyncActionNode
{
public:
  SetChassisVelocity(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr);

  static PortsList providedPorts();

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;
private:
  rclcpp::Publisher<Twist>::SharedPtr publisher_;

  float val;

  rclcpp::Node::SharedPtr node_ptr_;

};