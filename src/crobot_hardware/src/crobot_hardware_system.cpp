#include "crobot_hardware/crobot_hardware_system.hpp"

namespace crobot_hardware
{
    hardware_interface::CallbackReturn CrobotHardware::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        // TODO: Implement initialization
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> CrobotHardware::export_state_interfaces()
    {
        // TODO: Implement state interface export
        return std::vector<hardware_interface::StateInterface>();
    }

    std::vector<hardware_interface::CommandInterface> CrobotHardware::export_command_interfaces()
    {
        // TODO: Implement command interface export
        return std::vector<hardware_interface::CommandInterface>();
    }

    hardware_interface::CallbackReturn CrobotHardware::on_configure(
        const rclcpp_lifecycle::State & previous_state)
    {
        // TODO: Implement configuration
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_cleanup(
        const rclcpp_lifecycle::State & previous_state)
    {
        // TODO: Implement cleanup
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_activate(
        const rclcpp_lifecycle::State & previous_state)
    {
        // TODO: Implement activation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        // TODO: Implement deactivation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CrobotHardware::read(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // TODO: Implement read from Teensy
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CrobotHardware::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // TODO: Implement write to Teensy
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    crobot_hardware::CrobotHardware, hardware_interface::SystemInterface
)