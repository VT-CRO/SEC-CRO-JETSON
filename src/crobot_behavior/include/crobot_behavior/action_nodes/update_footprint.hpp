#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

/**
 * UpdateFootprint
 *
 * BT action node that publishes a new footprint to both Nav2 costmaps.
 * Use `extended = true`  before lowering the sweeper (adds ~90 mm forward).
 * Use `extended = false` after raising  the sweeper (restores original size).
 *
 * Input ports
 *   extended  [bool]  – true → sweeper-down footprint, false → normal footprint
 */
class UpdateFootprint : public BT::SyncActionNode
{
public:
    UpdateFootprint(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;

    // Two publishers – one per costmap namespace
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr local_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr global_pub_;

    // ── Footprint dimensions (metres) ──────────────────────────────────────
    // Measured from base_link (centre of robot).
    // Robot body:  ±0.1524 m left/right,  ±0.1524 m front/back
    // Sweeper adds ~0.090 m (≈ 3.5 in) to the forward (+ x) face only.
    static constexpr double kHalfWidth   = 0.1524;   // y  (left/right half)
    static constexpr double kHalfBack    = 0.1524;  // −x (rear half)
    static constexpr double kHalfFront   = 0.1524;  // +x (front half, normal)
    static constexpr double kSweeperExt  = 0.090;   // extra forward reach

    geometry_msgs::msg::PolygonStamped buildFootprint(bool extended) const;
};