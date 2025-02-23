#ifndef APRILTAG_SUBSCRIBER_HPP
#define APRILTAG_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <optional>
#include <thread>
#include <vector>
using namespace std;

class AprilTagSubscriberNode : public BT::SyncActionNode {
public:
    AprilTagSubscriberNode(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);
    ~AprilTagSubscriberNode();

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    thread spin_thread_;
    optional<int> last_detected_id_;
    string positions [5] = {"0.0;0.0;0.0", "1.0;2.0;2.0", "2.0;2.0;2.0", "3.0;2.0;2.0","4.0;2.0;2.0"};
};

#endif // APRILTAG_SUBSCRIBER_NODE_HPP
