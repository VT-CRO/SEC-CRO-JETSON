#ifndef CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"

namespace crobot_hardware
{
// TODO: Define message types

    // TODO: Better understand last year's implementation
    class CrobotHardware : public hardware_interface::SystemInterface
    {
        struct Config
        {
            // TODO: Add config
        };

        public:
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
            // TODO: Add private members
    };
}

#endif  // CROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_