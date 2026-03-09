#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <string>

// Nodes
#include "crobot_behavior/action_nodes/go_to_pose.hpp"
#include "crobot_behavior/action_nodes/press_button.hpp"   // <-- ADD THIS
#include "crobot_behavior/NavigationServer.hpp"
#include "crobot_behavior/action_nodes/sweeper_controls.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_launch");
  auto navigationNode = std::make_shared<NavigationServer>(rclcpp::NodeOptions());

  BT::BehaviorTreeFactory factory;

  // --- Register GoToPose ---
  factory.registerBuilder<GoToPose>(
    "GoToPose",
    [node](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GoToPose>(name, config, node);
    }
  );

  // --- Register SweeperControl ---
  factory.registerBuilder<SweeperControl>(
    "SweeperControl",
    [node](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SweeperControl>(name, config, node);
    }
  );

  // --- Register PressButton ---                          // <-- ADD THIS BLOCK
  factory.registerBuilder<PressButton>(
    "PressButton",
    [node](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<PressButton>(name, config, node);
    }
  );

  std::string pkg_path =
    ament_index_cpp::get_package_share_directory("crobot_behavior");

  std::string xml_path = pkg_path + "/trees/default.xml";

  auto tree = factory.createTreeFromFile(xml_path);

  rclcpp::Rate rate(20);
  while (rclcpp::ok())
  {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rclcpp::spin_some(navigationNode);
    rate.sleep();
  }

  rclcpp::shutdown();
}