#ifndef APRILTAG_ID_SUBSCRIBER_HPP
#define APRILTAG_ID_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <optional>
#include <thread>
#include <vector>
using namespace std;

class AprilTagSubscriberID : public BT::StatefulActionNode {
public:
    AprilTagSubscriberID(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);
    ~AprilTagSubscriberID();

    // BT::NodeStatus tick() override;
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    BT::NodeStatus onHalted() override;

    static BT::PortsList providedPorts();

private:
    void callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    optional<int> last_detected_id;
    string positions [5] = {"0.0;0.0;0.0", "1.0;1.0;1.0", "2.0;2.0;2.0", "3.0;3.0;3.0","4.0;4.0;4.0"};
    bool detection = false;
};

#endif // APRILTAG_ID_SUBSCRIBER_NODE_HPP
