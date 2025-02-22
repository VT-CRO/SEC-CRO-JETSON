#ifndef APRILTAG_SUBSCRIBER_HPP
#define APRILTAG_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <optional>
#include <thread>

class AprilTagSubscriberNode : public BT::SyncActionNode {
public:
    AprilTagSubscriberNode(const std::string &name, const BT::NodeConfiguration &config);
    ~AprilTagSubscriberNode();

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    void callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
    std::optional<int> last_detected_id_;
};

#endif // APRILTAG_SUBSCRIBER_NODE_HPP
