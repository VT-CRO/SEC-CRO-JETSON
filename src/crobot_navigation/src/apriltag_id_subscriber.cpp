#include "crobot_navigation/behaviors/apriltag_id_subscriber.hpp"
// using namespace std;

AprilTagSubscriberID::AprilTagSubscriberID(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_(node_ptr) {
    
    subscription_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", 10, std::bind(&AprilTagSubscriberID::callback, this, std::placeholders::_1));
}

AprilTagSubscriberID::~AprilTagSubscriberID() {
}

BT::NodeStatus AprilTagSubscriberID::onStart() {
    detection = false;
    last_detected_id.reset();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AprilTagSubscriberID::onRunning() {
    if (!detection) {
        return BT::NodeStatus::RUNNING;
    } 
    if (!last_detected_id.has_value()) {
        return BT::NodeStatus::FAILURE;
    } else {
        setOutput("id", last_detected_id.value());
        setOutput("position", positions[last_detected_id.value()]);
        return BT::NodeStatus::SUCCESS;
    }
}

void AprilTagSubscriberID::onHalted() {
    detection = false;
}

BT::PortsList AprilTagSubscriberID::providedPorts() {
    return {BT::OutputPort<int>("id"), BT::OutputPort<std::string>("positions")};
}

void AprilTagSubscriberID::callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    if (!msg->detections.empty()) {
        detection = true;
        last_detected_id = msg->detections[0].id;
        RCLCPP_INFO(node_->get_logger(), "Detected AprilTag ID: %d", last_detected_id.value());
    }
}


