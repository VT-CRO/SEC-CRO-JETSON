#include "crobot_hardware/crobot_hardware_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;

namespace crobot_hardware
{
    hardware_interface::CallbackReturn CrobotHardware::on_init(
        const hardware_interface::HardwareInfo & info
    )
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS
        )
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
        cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];
        cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
        cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];

        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["dev"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

        if (info_.hardware_parameters.count("pid_p") > 0)
        {
            cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
            cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
            cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
            cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "PID values not supplied, using defaults.");
        }

        wheel_back_left.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
        wheel_back_right.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);
        wheel_front_left.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
        wheel_front_right.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);


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

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CrobotHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY
                );

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CrobotHardware"),
                    "Joint '%s' has %zu state interfaces found. 2 expected.",
                    joint.name.c_str(),
                    joint.state_interfaces.size()
                );

                return hardware_interface::CallbackReturn::ERROR;
            }       

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CrobotHardware"),
                    "Joint '%s' have %s state interfaces found. '%s' expected.",
                    joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY
                );

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CrobotHardware"),
                    "Joint '%s' have %s state interfaces found. '%s' expected.",
                    joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY
                );

                return hardware_interface::CallbackReturn::ERROR;
            }     
        }

        for (const hardware_interface::ComponentInfo & sensor : info_.sensors)
        {
            RCLCPP_INFO(
                    rclcpp::get_logger("CrobotHardware"),
                    "Found sensor '%s'",
                    sensor.name.c_str()
                );
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> CrobotHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_back_left.name, hardware_interface::HW_IF_POSITION, &wheel_back_left.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_back_left.name, hardware_interface::HW_IF_VELOCITY, &wheel_back_left.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_back_right.name, hardware_interface::HW_IF_POSITION, &wheel_back_right.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_back_right.name, hardware_interface::HW_IF_VELOCITY, &wheel_back_right.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_front_left.name, hardware_interface::HW_IF_POSITION, &wheel_front_left.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_front_left.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_left.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_front_right.name, hardware_interface::HW_IF_POSITION, &wheel_front_right.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_front_right.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_right.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "deadwheel_odom", "deadwheel_odom_x", &deadwheels.pos_x
        ));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "deadwheel_odom", "deadwheel_odom_y", &deadwheels.pos_y
        ));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "deadwheel_odom", "deadwheel_odom_heading", &deadwheels.pos_th
        ));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> CrobotHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_back_left.name, hardware_interface::HW_IF_VELOCITY, &wheel_back_left.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_back_right.name, hardware_interface::HW_IF_VELOCITY, &wheel_back_right.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_front_left.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_left.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_front_right.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_right.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_configure(
        const rclcpp_lifecycle::State &
    )
    {
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Configuring ...please wait...");
        if (comms_.connected())
        {
            comms_.disconnect();
        }
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Attempting to connect to device %s...", cfg_.device.c_str());
        
        comms_.connect(cfg_.device, cfg_.timeout_ms);
        comms_.flush();
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_cleanup(
        const rclcpp_lifecycle::State &
    )
    {
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Cleaning up ...please wait...");
        if (comms_.connected())
        {
            comms_.disconnect();
        }
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Activating ...please wait...");
        if (!comms_.connected())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CrobotHardware"), "Could not connect to device!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (cfg_.pid_p > 0)
        {
            // comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
        }
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CrobotHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
        {
            RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Deactivating ...please wait...");
            RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CrobotHardware::read(
        const rclcpp::Time &, const rclcpp::Duration & /* period */
    )
    {
        static char read_buff[100];

        if (!comms_.connected())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CrobotHardware"), "Hardware not connected!");
            return hardware_interface::return_type::ERROR;
        }

        json j;
        j["header"]["message_type"] = REQUEST;

        std::string req = j.dump();

        comms_.writeBytes(req.c_str(), req.size());

        // read encoder values
        std::size_t n = comms_.readBytes(read_buff, 100);
        std::string s(read_buff);
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Read %ld bytes: %s", n, s.c_str());
        comms_.flush();

        j = json::parse(s, nullptr, false);

        if (!j.is_discarded())
        {
            deadwheels.pos_x = j["deadwheel_stats"]["x"];
            deadwheels.pos_y = j["deadwheel_stats"]["y"];
            deadwheels.pos_th = j["deadwheel_stats"]["heading"];
            start_led = j["start_led"];
        } else {
            RCLCPP_WARN(rclcpp::get_logger("CrobotHardware"), "Could not parse message!");
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CrobotHardware::write(
        const rclcpp::Time &, const rclcpp::Duration &
    )
    {
        if (!comms_.connected())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CrobotHardware"), "Hardware not connected!");
            return hardware_interface::return_type::ERROR;
        }

        json j;
        j["header"]["message_type"] = WRITE;

        j["motor_speeds"] = {
            wheel_front_right.cmd,
            wheel_front_left.cmd,
            wheel_back_right.cmd,
            wheel_back_left.cmd
        };

        j["lower_beacon"] = lower_beacon;
        j["run"] = run;
        j["bin_intake"] = bin_intake;

        std::string s = j.dump();

        comms_.writeBytes(s.c_str(), s.size());
        
        RCLCPP_INFO(rclcpp::get_logger("(CrobotHardware)"), "Sent Data: %s", s.c_str());

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    crobot_hardware::CrobotHardware, hardware_interface::SystemInterface
)