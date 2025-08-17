#include "crobot_navigation/behaviors/apriltag_id_subscriber.hpp"
#include "crobot_navigation/behaviors/odom_go_to_position.hpp"


AprilTagSubscriberID::AprilTagSubscriberID(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_(node_ptr) {
}

AprilTagSubscriberID::~AprilTagSubscriberID() {
}

// NavGoal MakeNavGoal(float x, float y, float th)
// {
//     auto navGoal = NavPose::Goal();
//     navGoal.pose.header.frame_id = "map";
//     navGoal.pose.pose.position.x = x;
//     navGoal.pose.pose.position.y = y;

//     tf2::Quaternion q;
//     q.setRPY(0, 0, th);
//     navGoal.pose.pose.orientation = tf2::toMsg(q);

//     return navGoal;
// }

void AprilTagSubscriberID::preparePoints() {
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

