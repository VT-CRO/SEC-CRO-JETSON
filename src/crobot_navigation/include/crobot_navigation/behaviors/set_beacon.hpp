// Example of custom SyncActionNode (synchronous action)
// without ports.

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"

class SetBeacon : public BT::SyncActionNode
{
public:
  SetBeacon(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("setBeacon")};
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;

private:
  using DynamicInterface = control_msgs::msg::DynamicInterfaceGroupValues;
  using InterfaceValue = control_msgs::msg::InterfaceValue;

  rclcpp::Publisher<DynamicInterface>::SharedPtr publisher_;

  float val;  // Ensure val is declared as float

  rclcpp::Node::SharedPtr node_ptr_;

};