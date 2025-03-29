#include "crobot_navigation/behaviors/apriltag_id_subscriber.hpp"
#include "crobot_navigation/behaviors/odom_go_to_position.hpp"


AprilTagSubscriberID::AprilTagSubscriberID(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_(node_ptr) {
}

AprilTagSubscriberID::~AprilTagSubscriberID() {
}

void AprilTagSubscriberID::preparePoints() {
    // TODO: Set april tag points here
    positions.push_back({10, 215, 0});
    positions.push_back({33, 215, 0});
    positions.push_back({56, 215, 0});
    positions.push_back({79, 215, 0});
    positions.push_back({102, 215, 0});
}

BT::NodeStatus AprilTagSubscriberID::onStart() {
    subscription_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", 10, std::bind(&AprilTagSubscriberID::callback, this, std::placeholders::_1));

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
        RCLCPP_INFO(node_->get_logger(), "Detected AprilTag ID: %d", last_detected_id.value());
        setOutput("id", last_detected_id.value());
        setOutput("positions", positions[last_detected_id.value()]);
        return BT::NodeStatus::SUCCESS;
    }
}

void AprilTagSubscriberID::onHalted() {
    detection = false;
}

BT::PortsList AprilTagSubscriberID::providedPorts() {
    return {BT::OutputPort<int>("id"), BT::OutputPort<NavPointsGoal>("positions")};
}

void AprilTagSubscriberID::callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    if (!msg->detections.empty()) {
        detection = true;
        last_detected_id = msg->detections[0].id;
    }
}

