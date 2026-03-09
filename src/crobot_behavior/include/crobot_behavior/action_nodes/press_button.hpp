#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

// ============================================================================
// PressButton
//
// Drives the robot into the button repeatedly, then backs away.
//
// State machine (runs inside onRunning each tick):
//
//   PRESSING_FORWARD  ──(time elapsed)──►  PRESSING_BACKWARD
//        ▲                                        │
//        └────────────(repeat_count < reps)───────┘
//                                                 │
//                                  (repeat_count >= reps)
//                                                 │
//                                                 ▼
//                                          BACKING_UP
//                                                 │
//                                        (time elapsed)
//                                                 │
//                                                 ▼
//                                             DONE ──► SUCCESS
//
// Ports (all optional — sensible defaults are provided):
//   forward_speed   (m/s)  Speed to drive into the button. Default: 0.08
//   backward_speed  (m/s)  Speed to back away between presses. Default: 0.10
//   forward_secs    (s)    How long to drive into button each press. Default: 0.4
//   backward_secs   (s)    How long to reverse between presses. Default: 0.4
//   backup_secs     (s)    How long to reverse after all presses. Default: 1.2
//   num_presses     (int)  Number of button presses. Default: 3
// ============================================================================

class PressButton : public BT::StatefulActionNode
{
public:
    PressButton(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node_ptr
    );

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    // Internal state machine phases
    enum class Phase {
        PRESSING_FORWARD,
        PRESSING_BACKWARD,
        BACKING_UP,
        DONE
    };

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // State
    Phase  phase_         = Phase::PRESSING_FORWARD;
    int    press_count_   = 0;
    int    num_presses_   = 3;
    rclcpp::Time phase_start_time_;

    // Tunable parameters read from ports
    double forward_speed_  = 0.08;
    double backward_speed_ = 0.10;
    double forward_secs_   = 0.4;
    double backward_secs_  = 0.4;
    double backup_secs_    = 1.2;

    void publishVelocity(double linear_x);
    void stopRobot();
};
