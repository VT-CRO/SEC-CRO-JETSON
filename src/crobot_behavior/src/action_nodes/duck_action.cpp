#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class IsDuckAhead : public BT::ConditionNode
{
public:
  IsDuckAhead(const std::string & name, const BT::NodeConfig & config,
              rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node)
  {
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/duck_detected", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        duck_detected_ = msg->data;
      });
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    return duck_detected_ ? BT::NodeStatus::SUCCESS
                          : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool duck_detected_ = false;
};