#include "crobot_behavior/action_nodes/update_footprint.hpp"
#include <geometry_msgs/msg/point32.hpp>

// ── Constructor ─────────────────────────────────────────────────────────────
UpdateFootprint::UpdateFootprint(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node_ptr)
: BT::SyncActionNode(name, config), node_ptr_(node_ptr)
{
    // Nav2 costmap nodes subscribe to "<namespace>/footprint"
    local_pub_  = node_ptr_->create_publisher<geometry_msgs::msg::Polygon>(
        "/local_costmap/footprint",  rclcpp::SystemDefaultsQoS());
    global_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Polygon>(
        "/global_costmap/footprint", rclcpp::SystemDefaultsQoS());
}

// ── Port list ────────────────────────────────────────────────────────────────
BT::PortsList UpdateFootprint::providedPorts()
{
    return {
        BT::InputPort<bool>("extended",
            "true  = sweeper-down footprint (adds ~50 mm forward)\n"
            "false = normal footprint")
    };
}

// ── Tick ─────────────────────────────────────────────────────────────────────
BT::NodeStatus UpdateFootprint::tick()
{
    bool extended = false;
    auto result = getInput("extended", extended);
    if (!result) {
        throw BT::RuntimeError("UpdateFootprint: missing required port [extended] – ", result.error());
    }

    auto msg = buildFootprint(extended);

    // Publish to both costmaps; Nav2 applies the update on the next costmap cycle
    local_pub_->publish(msg);
    global_pub_->publish(msg);

    RCLCPP_INFO(node_ptr_->get_logger(),
        "[UpdateFootprint] Published %s footprint (front reach: %.3f m)",
        extended ? "EXTENDED" : "NORMAL",
        extended ? kHalfFront + kSweeperExt : kHalfFront);

    return BT::NodeStatus::SUCCESS;
}

// ── Helper ───────────────────────────────────────────────────────────────────
geometry_msgs::msg::Polygon
UpdateFootprint::buildFootprint(bool extended) const
{
    geometry_msgs::msg::Polygon fp;
    // fp.header.stamp    = node_ptr_->now();
    // fp.header.frame_id = "base_link";

    const double front = kHalfFront + (extended ? kSweeperExt : 0.0);

    // Corners in CCW order (Nav2 convention):
    //   back-left → front-left → front-right → back-right
    //
    //        front (+x)
    //   BL ──────────── FL
    //   |                |
    //   BR ──────────── FR
    //        back  (-x)

    auto pt = [](double x, double y) {
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = 0.0f;
        return p;
    };

    fp.points = {
        pt(-kHalfBack,  -kHalfWidth),  // back-left
        pt( front,      -kHalfWidth),  // front-left
        pt( front,       kHalfWidth),  // front-right
        pt(-kHalfBack,   kHalfWidth),  // back-right
    };

    return fp;
}