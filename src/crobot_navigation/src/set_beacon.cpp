#include "crobot_navigation/behaviors/set_beacon.hpp"

SetBeacon::SetBeacon(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) :
      BT::SyncActionNode(name, config),  // Fix the constructor initialization list
      node_ptr_(node_ptr)
{
    publisher_ = node_ptr_-> create_publisher<DynamicInterface>("/gpio_controller/commands", 10);
}

// static BT::PortsList SetBeacon::providedPorts() 

  BT::NodeStatus SetBeacon::tick() 
  {
    auto setBeacon = getInput<std::string>("setBeacon");

    if ( !setBeacon ) {
      throw BT::RuntimeError("error reading port [setBeacon]: ", setBeacon.error());
    }

    if (setBeacon.value() == "true") {
      val = 1;
    } else {
      val = 0;
    }

    auto msg = DynamicInterface();
    msg.interface_groups = {"crobot_systems"};

    auto interface = InterfaceValue();
    interface.interface_names = {"beacon"};

    interface.values = {val};
    msg.interface_values = {interface};

    publisher_-> publish(msg);

    return BT::NodeStatus::SUCCESS;
  }