#include "crobot_hardware/crobot_hardware_system.hpp"

#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crobot_hardware
{
    hardware_interface::CallbackReturn CrobotHardware::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.wheel_fl_name = info_.hardware_parameters["front_left_wheel_name"];
        cfg_.wheel_fr_name = info_.hardware_parameters["front_right_wheel_name"];
        cfg_.wheel_bl_name = info_.hardware_parameters["back_left_wheel_name"];
        cfg_.wheel_br_name = info_.hardware_parameters["back_right_wheel_name"];

        cfg_.ankle_fl_name = info_.hardware_parameters["front_left_ankle_name"];
        cfg_.ankle_fr_name = info_.hardware_parameters["front_right_ankle_name"];
        cfg_.ankle_bl_name = info_.hardware_parameters["back_left_ankle_name"];
        cfg_.ankle_br_name = info_.hardware_parameters["back_right_ankle_name"];

        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["dev"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

        wheels_.resize(4);
        wheels_[0].name = cfg_.wheel_fl_name;
        wheels_[1].name = cfg_.wheel_fr_name;
        wheels_[2].name = cfg_.wheel_bl_name;
        wheels_[3].name = cfg_.wheel_br_name;

        ankles_.resize(4);
        ankles_[0].name = cfg_.ankle_fl_name;
        ankles_[1].name = cfg_.ankle_fr_name;
        ankles_[2].name = cfg_.ankle_bl_name;
        ankles_[3].name = cfg_.ankle_br_name;

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CrobotHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(),
                    joint.command_interfaces.size()
                );

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CrobotHardware"),
                    "Joint '%s' has %zu state interfaces found. 1 expected.",
                    joint.name.c_str(),
                    joint.state_interfaces.size()
                );

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.name.find("ankle") != std::string::npos)
            {
                if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("CrobotHardware"),
                        "Joint '%s' has '%s' command interface. '%s' expected.",
                        joint.name.c_str(),
                        joint.command_interfaces[0].name.c_str(),
                        hardware_interface::HW_IF_POSITION
                    );

                    return hardware_interface::CallbackReturn::ERROR;
                }

                if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("CrobotHardware"),
                        "Joint '%s' has '%s' state interface. '%s' expected.",
                        joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(),
                        hardware_interface::HW_IF_POSITION
                    );

                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
            else
            {
                if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("CrobotHardware"),
                        "Joint '%s' has '%s' command interface. '%s' expected.",
                        joint.name.c_str(),
                        joint.command_interfaces[0].name.c_str(),
                        hardware_interface::HW_IF_VELOCITY
                    );

                    return hardware_interface::CallbackReturn::ERROR;
                }

                if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("CrobotHardware"),
                        "Joint '%s' has '%s' state interface. '%s' expected.",
                        joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(),
                        hardware_interface::HW_IF_VELOCITY
                    );

                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> CrobotHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto & wheel : wheels_)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.vel
            ));
        }

        for (auto & ankle : ankles_)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                ankle.name, hardware_interface::HW_IF_POSITION, &ankle.pos
            ));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> CrobotHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto & wheel : wheels_)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.cmd
            ));
        }

        for (auto & ankle : ankles_)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                ankle.name, hardware_interface::HW_IF_POSITION, &ankle.cmd
            ));
        }
        
        return command_interfaces;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_configure(
        const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Configuring...");

        if (!serial_comm_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("CrobotHardware"),
                "Failed to connect to device '%s' at %d baud",
                cfg_.device.c_str(),
                cfg_.baud_rate
            );

            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_cleanup(
        const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Cleaning up...");
    
        serial_comm_.disconnect();

        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Cleaned up.");

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