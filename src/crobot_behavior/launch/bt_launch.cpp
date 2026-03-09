#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <string>

// Nodes
#include "crobot_behavior/action_nodes/go_to_pose.hpp"
#include "crobot_behavior/action_nodes/press_button.hpp"   // <-- ADD THIS
#include "crobot_behavior/NavigationServer.hpp"
#include "crobot_behavior/action_nodes/sweeper_controls.hpp"
#include "geometry_msgs/msg/twist.hpp"


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
  
  while (rclcpp::ok()) {
    BT::NodeStatus status = tree.tickRoot();

    rclcpp::spin_some(node);
    rclcpp::spin_some(navigationNode);

    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      geometry_msgs::msg::Twist stop_msg;

      stop_msg.linear.x = 0.0;
      stop_msg.linear.y = 0.0;
      stop_msg.linear.z = 0.0;
      stop_msg.angular.x = 0.0;
      stop_msg.angular.y = 0.0;
      stop_msg.angular.z = 0.0;

      // stop!!!!!!!!
      for (int i = 0; i < 5; ++i) {
        stop_pub->publish(stop_msg);
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }

      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Behavior tree completed successfully. Stopping...");
      } else {
        RCLCPP_WARN(node->get_logger(), "Behavior tree failed. Stopping...");
      }

      break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}