#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class TurnCrank : public BT::StatefulActionNode
{
public:
    TurnCrank(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted()  override;

private:
    enum class Phase { TURNING, DONE };

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr winch_pub_;

    Phase phase_ = Phase::TURNING;
    rclcpp::Time start_time_;

    double run_seconds_ = 10.0;
    double fwd_speed_  = 0.1;
    double right_speed_  = -0.1; // -y, so right
    double winch_speed_  = 1.0;

    void stopRobot();
};