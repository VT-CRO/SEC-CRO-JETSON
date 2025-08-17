#include "crobot_navigation/behaviors/set_sorting.hpp"

SetSorting::SetSorting(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) :
      BT::SyncActionNode(name, config),  // Fix the constructor initialization list
      node_ptr_(node_ptr)
{
    publisher_ = node_ptr_-> create_publisher<DynamicInterface>("/gpio_controller/commands", 10);
}

// static BT::PortsList SetBeacon::providedPorts() 

  BT::NodeStatus SetSorting::tick() 
  {
    auto setSorting = getInput<std::string>("setSorting");

    if ( !setSorting ) {
      throw BT::RuntimeError("error reading port [setBeacon]: ", setSorting.error());
    }

    if (setSorting.value() == "center") {
      val = 0;
    } else if (setSorting.value() == "geo"){
      val = 1;
    } else if (setSorting.value() == "neb"){
      val = 2;
    } else {
      throw BT::RuntimeError("invalid input!");
    }

    auto msg = DynamicInterface();
    msg.interface_groups = {"crobot_systems"};

    auto interface = InterfaceValue();
    interface.interface_names = {"sorting"};

    interface.values = {val};
    msg.interface_values = {interface};

    publisher_-> publish(msg);

    return BT::NodeStatus::SUCCESS;
  }