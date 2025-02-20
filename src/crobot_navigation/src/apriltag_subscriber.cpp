#include "apriltag_subscriber.hpp"

AprilTagSubscriber::AprilTagSubscriber(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(std::make_shared<rclcpp::Node>("april_tag_subscriber"))
{
    subscription_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/detections", 10, std::bind(&AprilTagSubscriber::topic_callback, this, std::placeholders::_1));
}

BT::PortsList AprilTagSubscriber::providedPorts() {
    return { BT::OutputPort<std::vector<int>>("detected_ids") };
}

BT::NodeStatus AprilTagSubscriber::tick() {
    if (!last_detected_ids_.empty()) {
        setOutput("detected_ids", last_detected_ids_);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void AprilTagSubscriber::topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    last_detected_ids_ = msg->data;
}
