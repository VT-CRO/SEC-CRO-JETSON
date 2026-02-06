#ifndef CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "crobot_hardware/serial_comm.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crobot_hardware
{
    struct Motor
    {
        std::string name;
        // double pos = 0.0;
        double vel = 0.0;
        double cmd = 0.0;
    };

    struct Servo
    {
        std::string name;
        double pos = 0.0;
        double cmd = 0.0;
    };

    class CrobotHardware : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(CrobotHardware)

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
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

                float loop_rate = 0.0;
                std::string device = "";
                int baud_rate = 115200;
                int timeout_ms = 1000;
            } cfg_;

            // fl, fr, bl, br)
            std::vector<Motor> wheels_;
            std::vector<Servo> ankles_;

            Servo sweeper_;

            SerialComm serial_comm_;
    };
}

#endif  // CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_