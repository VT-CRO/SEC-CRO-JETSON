#ifndef CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "crobot_hardware/serial_comm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace crobot_hardware
{
    struct Motor
    {
        std::string name;
        double pos = 0.0;
        double vel = 0.0;
        double cmd = 0.0;
    };

    struct Servo
    {
        std::string name;
        double pos = 0.0;
        double cmd = 0.0;
    };

    double imu_vel = 0.0;

    class CrobotHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(CrobotHardware)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        struct Config
        {
            std::string wheel_fl_name;
            std::string wheel_fr_name;
            std::string wheel_bl_name;
            std::string wheel_br_name;

            std::string ankle_fl_name;
            std::string ankle_fr_name;
            std::string ankle_bl_name;
            std::string ankle_br_name;

            std::string sweeper_name;
            std::string winch_name;

            // Added what I think is needed here for the arm servos and the flag servo
            std::string shoulder_name;
            std::string elbow_name;
            std::string gripper_name;
            std::string flagdropper_name;

            std::string imu_name;

            float max_wheel_speed_meters = 0.36; // m/s corresponding to full command (255)
            float wheel_radius = 0.035;          // meters

            float loop_rate = 0.0;
            std::string device = "";
            int baud_rate = 115200;
            int timeout_ms = 1000;
        } cfg_;

        // fl, fr, bl, br)
        std::vector<Motor> wheels_;
        std::vector<Servo> ankles_;

        Servo sweeper_;
        Motor winch_;

        // Defined the four new servos
        Servo shoulder_;
        Servo elbow_;
        Servo gripper_;
        Servo flagdropper_;

        SerialComm serial_comm_;

        bool first_read_ = true;
        int32_t last_ticks_fl_ = 0;
        int32_t last_ticks_fr_ = 0;
        // int32_t last_ticks_br_ = 0;

        rclcpp::Node::SharedPtr imu_node_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        double imu_yaw_bias_ = 0.0;
        int bias_sample_count_ = 0;
        static constexpr int BIAS_SAMPLES = 60; // ~2 seconds at 100Hz
        bool bias_calibrated_ = false;
    };
}

#endif // CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_