#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

class TeleopMapper : public rclcpp::Node
{
public:
    TeleopMapper() : Node("teleop_mapper")
    {
        // the 10 is default behavior for QoS queue size from ros 1
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TeleopMapper::cmdVelCallback, this, std::placeholders::_1));

        pub_ankles_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/ankle_position_controller/commands", 10);
        pub_wheels_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/wheel_velocity_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(), "Teleop Mapper Node Started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        std_msgs::msg::Float64MultiArray ankle_msg;
        std_msgs::msg::Float64MultiArray wheel_msg;

        std::vector<double> ankles(4, 0.0);
        std::vector<double> wheels(4, 0.0);

        const double MAX_ANGLE = M_PI / 4.0;

        if (std::abs(linear_x) < 0.01 && std::abs(angular_z) > 0.01)
        {
            // positive angular velocity is ccw, negative is cw
            double s = (angular_z > 0) ? 1.0 : -1.0;
            double v = std::abs(angular_z);

            ankles[0] = -MAX_ANGLE;
            ankles[1] = MAX_ANGLE;
            ankles[2] = MAX_ANGLE;
            ankles[3] = -MAX_ANGLE;

            wheels[0] = -v * s;
            wheels[1] = -v * s;
            wheels[2] = v * s;
            wheels[3] = v * s;
        }
        else
        {
            double steer = std::max(-MAX_ANGLE, std::min(angular_z, MAX_ANGLE));

            ankles[0] = steer;
            ankles[1] = steer;
            ankles[2] = -steer;
            ankles[3] = -steer;

            for(int i = 0; i < 4; i++) {
                wheels[i] = linear_x;
            }
        }

        ankle_msg.data = ankles;
        wheel_msg.data = wheels;

        pub_ankles_->publish(ankle_msg);
        pub_wheels_->publish(wheel_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ankles_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheels_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopMapper>());
    rclcpp::shutdown();
    return 0;
}
