#ifndef APRILTAG_SUBSCRIBER_HPP
#define APRILTAG_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <behaviortree_cpp/bt_factory.h>

class AprilTagSubscriber : public BT::SyncActionNode {
public:
    AprilTagSubscriber(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    std::vector<int> last_detected_ids_;
};

#endif // APRILTAG_SUBSCRIBER_HPP
