#include "crobot_navigation/behaviors/set_bin_intake.hpp"

SetBinIntake::SetBinIntake(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr) :
      BT::SyncActionNode(name, config),  // Fix the constructor initialization list
      node_ptr_(node_ptr)
{
    publisher_ = node_ptr_-> create_publisher<DynamicInterface>("/gpio_controller/commands", 10);
}

// static BT::PortsList SetBeacon::providedPorts() 

  BT::NodeStatus SetBinIntake::tick() 
  {
    auto setBinIntake = getInput<std::string>("setBinIntake");

    if ( !setBinIntake ) {
      throw BT::RuntimeError("error reading port [setBinIntake]: ", setBinIntake.error());
    }

    if (setBinIntake.value() == "in") {
      val = 0;
     } else if (setBinIntake.value() == "out") {
      val = 1;
    } else if (setBinIntake.value() == "off") {
      val = 2;
    } else {
      throw BT::RuntimeError("invalid input!");
    }

    auto msg = DynamicInterface();
    msg.interface_groups = {"crobot_systems"};

    auto interface = InterfaceValue();
    interface.interface_names = {"bin_intake"};

    interface.values = {val};
    msg.interface_values = {interface};

    publisher_-> publish(msg);

    return BT::NodeStatus::SUCCESS;
  }