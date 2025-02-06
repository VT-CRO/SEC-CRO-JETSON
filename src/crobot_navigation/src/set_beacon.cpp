#include "crobot_navigation/behaviors/set_beacon.hpp"

SetBeacon::SetBeacon(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) :
      BT::SyncActionNode(name, {}),
      node_ptr_(node_ptr)
{
    publisher_ = node_ptr_-> create_publisher<DynamicInterface>("/gpio_controller/commands", 10);
}

  BT::NodeStatus SetBeacon::tick() 
  {
    auto msg = DynamicInterface();
    msg.interface_groups = {"crobot_systems"};

    auto interface = InterfaceValue();
    interface.interface_names = {"bin_intake"};

    interface.values = {val};
    msg.interface_values = {interface};

    publisher_-> publish(msg);

    return BT::NodeStatus::SUCCESS;
  } 