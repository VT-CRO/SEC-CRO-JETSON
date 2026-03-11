#include "crobot_behavior/action_nodes/turn_crank.hpp"

TurnCrank::TurnCrank(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    cmd_vel_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    winch_pub_  = node_ptr_->create_publisher<std_msgs::msg::Float64>("/winch_velocity_controller/commands", 10);
}

BT::PortsList TurnCrank::providedPorts()
{
    return {
        BT::InputPort<double>("run_seconds", 10.0, "Seconds to turn the crank"),
        BT::InputPort<double>("fwd_speed", 0.05, "Forward velfocity (x)"),
        BT::InputPort<double>("right_speed", -0.05, "Rightward velocity (y)"),
        BT::InputPort<double>("winch_speed", 1.0,  "Winch motor velocity")
    };
}

BT::NodeStatus TurnCrank::onStart()
{
    getInput("run_seconds", run_seconds_);
    getInput("fwd_speed", fwd_speed_);
    getInput("right_speed", right_speed_);
    getInput("winch_speed", winch_speed_);

    phase_ = Phase::TURNING;
    start_time_ = node_ptr_->now();

    RCLCPP_INFO(node_ptr_->get_logger(), "[TurnCrank] Starting for %.1fs", run_seconds_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnCrank::onRunning()
{
    double elapsed = (node_ptr_->now() - start_time_).seconds();

    if (phase_ == Phase::TURNING)
    {
        if (elapsed >= run_seconds_) {
            stopRobot();
            phase_ = Phase::DONE;
        }
        else {
            // diagonal motion
            geometry_msgs::msg::Twist drive_msg;
            drive_msg.linear.x = fwd_speed_;
            drive_msg.linear.y = right_speed_; // strafe right
            cmd_vel_pub_->publish(drive_msg);

            std_msgs::msg::Float64 winch_msg;
            winch_msg.data = {winch_speed_}; 
            winch_pub_->publish(winch_msg);
        }
    }

    if (phase_ == Phase::DONE) return BT::NodeStatus::SUCCESS;
    
    return BT::NodeStatus::RUNNING;
}

void TurnCrank::onHalted()
{
    stopRobot();
}

void TurnCrank::stopRobot()
{
    // stop wheels
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    
    // stop winch
    std_msgs::msg::Float64 stop_winch;
    stop_winch.data = {0.0};
    winch_pub_->publish(stop_winch);
}