#include "crobot_behavior/action_nodes/press_button.hpp"

// ============================================================================
// Constructor
// ============================================================================

PressButton::PressButton(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    // Publishes directly to the cmd_vel topic — bypasses Nav2 entirely.
    // This is intentional: we want raw, predictable motion, not path planning.
    cmd_vel_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

// ============================================================================
// Ports
// All ports are optional with defaults so the XML stays clean.
// ============================================================================

BT::PortsList PressButton::providedPorts()
{
    return {
        BT::InputPort<double>("forward_speed",  0.08, "Speed into button (m/s)"),
        BT::InputPort<double>("backward_speed", 0.10, "Speed backing off between presses (m/s)"),
        BT::InputPort<double>("forward_secs",   0.4,  "Seconds to hold forward each press"),
        BT::InputPort<double>("backward_secs",  0.4,  "Seconds to reverse between presses"),
        BT::InputPort<double>("backup_secs",    1.2,  "Seconds to back away after all presses"),
        BT::InputPort<int>   ("num_presses",    3,    "Number of button presses")
    };
}

// ============================================================================
// onStart — called once when the BT first activates this node
// ============================================================================

BT::NodeStatus PressButton::onStart()
{
    // Read parameters from BT ports (XML attributes), falling back to defaults
    getInput("forward_speed",  forward_speed_);
    getInput("backward_speed", backward_speed_);
    getInput("forward_secs",   forward_secs_);
    getInput("backward_secs",  backward_secs_);
    getInput("backup_secs",    backup_secs_);
    getInput("num_presses",    num_presses_);

    // Reset state so the node works correctly if the BT ever re-activates it
    press_count_     = 0;
    phase_           = Phase::PRESSING_FORWARD;
    phase_start_time_ = node_ptr_->now();

    RCLCPP_INFO(node_ptr_->get_logger(),
        "[PressButton] Starting: %d presses, fwd=%.2fs @ %.3fm/s, bwd=%.2fs @ %.3fm/s, backup=%.2fs",
        num_presses_, forward_secs_, forward_speed_,
        backward_secs_, backward_speed_, backup_secs_);

    return BT::NodeStatus::RUNNING;
}

// ============================================================================
// onRunning — called every BT tick (~50ms) while this node is active.
//
// This is the state machine. Each phase publishes a velocity command and
// waits until enough wall-clock time has elapsed, then transitions.
// ============================================================================

BT::NodeStatus PressButton::onRunning()
{
    double elapsed = (node_ptr_->now() - phase_start_time_).seconds();

    switch (phase_)
    {
        // ------------------------------------------------------------------
        case Phase::PRESSING_FORWARD:
        {
            // Keep driving into the button
            publishVelocity(forward_speed_);

            if (elapsed >= forward_secs_)
            {
                press_count_++;
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[PressButton] Press %d/%d complete — reversing", press_count_, num_presses_);

                stopRobot();

                if (press_count_ >= num_presses_)
                {
                    // All presses done — do the final back-up
                    phase_ = Phase::BACKING_UP;
                }
                else
                {
                    // More presses to go — reverse briefly before the next one
                    phase_ = Phase::PRESSING_BACKWARD;
                }
                phase_start_time_ = node_ptr_->now();
            }
            break;
        }

        // ------------------------------------------------------------------
        case Phase::PRESSING_BACKWARD:
        {
            // Brief reverse between presses so the button can spring back
            publishVelocity(-backward_speed_);

            if (elapsed >= backward_secs_)
            {
                RCLCPP_INFO(node_ptr_->get_logger(), "[PressButton] Ready for next press");
                stopRobot();
                phase_            = Phase::PRESSING_FORWARD;
                phase_start_time_ = node_ptr_->now();
            }
            break;
        }

        // ------------------------------------------------------------------
        case Phase::BACKING_UP:
        {
            // Final back-up to clear the button area before the next task
            publishVelocity(-backward_speed_);

            if (elapsed >= backup_secs_)
            {
                RCLCPP_INFO(node_ptr_->get_logger(), "[PressButton] Done — backed away, task complete");
                stopRobot();
                phase_ = Phase::DONE;
            }
            break;
        }

        // ------------------------------------------------------------------
        case Phase::DONE:
        {
            // Returning SUCCESS here hands control back to the parent Sequence,
            // which will then move on to the next sibling node (your next task).
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::RUNNING;
}

// ============================================================================
// onHalted — called if the BT cancels this node (e.g., a parallel watchdog
// fires, or the parent Sequence fails). Always stop the robot.
// ============================================================================

void PressButton::onHalted()
{
    RCLCPP_WARN(node_ptr_->get_logger(), "[PressButton] Halted — stopping robot");
    stopRobot();
    phase_ = Phase::PRESSING_FORWARD;  // Reset so re-activation starts fresh
}

// ============================================================================
// Helpers
// ============================================================================

void PressButton::publishVelocity(double linear_x)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = linear_x;
    msg.linear.y  = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
}

void PressButton::stopRobot()
{
    publishVelocity(0.0);
}
