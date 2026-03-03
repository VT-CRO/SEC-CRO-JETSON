#include "crobot_hardware/crobot_hardware_system.hpp"

#include <string>
#include <algorithm>
#include <nlohmann/json.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;

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

        cfg_.sweeper_name = info_.hardware_parameters["sweeper_name"];
        cfg_.winch_name = info_.hardware_parameters["winch_name"];

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

        sweeper_.name = cfg_.sweeper_name;
        winch_.name = cfg_.winch_name;

        // for (const hardware_interface::ComponentInfo & joint : info_.joints)
        // {
        //     if (joint.command_interfaces.size() != 1)
        //     {
        //         RCLCPP_FATAL(
        //             rclcpp::get_logger("CrobotHardware"),
        //             "Joint '%s' has %zu command interfaces found. 1 expected.",
        //             joint.name.c_str(),
        //             joint.command_interfaces.size()
        //         );

        //         return hardware_interface::CallbackReturn::ERROR;
        //     }

        //     if (joint.state_interfaces.size() != 1)
        //     {
        //         RCLCPP_FATAL(
        //             rclcpp::get_logger("CrobotHardware"),
        //             "Joint '%s' has %zu state interfaces found. 1 expected.",
        //             joint.name.c_str(),
        //             joint.state_interfaces.size()
        //         );

        //         return hardware_interface::CallbackReturn::ERROR;
        //     }

        //     if (joint.name.find("ankle") != std::string::npos || joint.name.find("sweeper") != std::string::npos)
        //     {
        //         if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        //         {
        //             RCLCPP_FATAL(
        //                 rclcpp::get_logger("CrobotHardware"),
        //                 "Joint '%s' has '%s' command interface. '%s' expected.",
        //                 joint.name.c_str(),
        //                 joint.command_interfaces[0].name.c_str(),
        //                 hardware_interface::HW_IF_POSITION
        //             );

        //             return hardware_interface::CallbackReturn::ERROR;
        //         }

        //         if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        //         {
        //             RCLCPP_FATAL(
        //                 rclcpp::get_logger("CrobotHardware"),
        //                 "Joint '%s' has '%s' state interface. '%s' expected.",
        //                 joint.name.c_str(),
        //                 joint.state_interfaces[0].name.c_str(),
        //                 hardware_interface::HW_IF_POSITION
        //             );

        //             return hardware_interface::CallbackReturn::ERROR;
        //         }
        //     }
        //     else if (joint.name.find("wheel") != std::string::npos || joint.name.find("winch") != std::string::npos)
        //     {
        //         if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        //         {
        //             RCLCPP_FATAL(
        //                 rclcpp::get_logger("CrobotHardware"),
        //                 "Joint '%s' has '%s' command interface. '%s' expected.",
        //                 joint.name.c_str(),
        //                 joint.command_interfaces[0].name.c_str(),
        //                 hardware_interface::HW_IF_VELOCITY
        //             );

        //             return hardware_interface::CallbackReturn::ERROR;
        //         }

        //         if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        //         {
        //             RCLCPP_FATAL(
        //                 rclcpp::get_logger("CrobotHardware"),
        //                 "Joint '%s' has '%s' state interface. '%s' expected.",
        //                 joint.name.c_str(),
        //                 joint.state_interfaces[0].name.c_str(),
        //                 hardware_interface::HW_IF_VELOCITY
        //             );

        //             return hardware_interface::CallbackReturn::ERROR;
        //         }
        //     }
        // }

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
            
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                wheel.name, hardware_interface::HW_IF_POSITION, &wheel.pos
            ));
        }

        for (auto & ankle : ankles_)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                ankle.name, hardware_interface::HW_IF_POSITION, &ankle.pos
            ));
        }

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sweeper_.name, hardware_interface::HW_IF_POSITION, &sweeper_.pos
        ));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            winch_.name, hardware_interface::HW_IF_VELOCITY, &winch_.vel
        ));

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

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            sweeper_.name, hardware_interface::HW_IF_POSITION, &sweeper_.cmd
        ));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            winch_.name, hardware_interface::HW_IF_VELOCITY, &winch_.cmd
        ));
        
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
        if (!serial_comm_.isConnected()) {
            return hardware_interface::return_type::ERROR;
        }

        json j;
        j["cmd"] = "read";
        std::string j_str = j.dump() + "\n";
        serial_comm_.writeBytes(j_str.c_str(), j_str.size());

        char buffer[256];
        int bytesRead = serial_comm_.readBytes(buffer, sizeof(buffer) - 1);

        if (bytesRead > 0) {
            // buffer[bytesRead] = '\0';   
            try {
                // wheels_[0].vel = response["wheels"]["front_left"];
                // wheels_[1].vel = response["wheels"]["front_right"];
                // wheels_[2].vel = response["wheels"]["back_left"];
                // wheels_[3].vel = response["wheels"]["back_right"];

                // ankles_[0].pos = response["ankles"]["front_left"];
                // ankles_[1].pos = response["ankles"]["front_right"];
                // ankles_[2].pos = response["ankles"]["back_left"];
                // ankles_[3].pos = response["ankles"]["back_right"];

                // stuff olivia added/changed
                std::string raw_data(buffer, bytesRead);                
                json response = json::parse(raw_data);

                if (response.contains("encoders")) {
                    static bool first_read = true;
                    std::cout << "the encoders should work" << std::endl; 
                    static int32_t last_ticks_fl = 0;
                    static int32_t last_ticks_fr = 0;
                    static int32_t last_ticks_br = 0;

                    const double COUNTS_PER_REV = 4096.0;
                    const double TWO_PI = 2.0 * M_PI;
                    const double dt = period.seconds();

                    int32_t ticks_fl = response["encoders"]["front_left"];
                    int32_t ticks_fr = response["encoders"]["front_right"];
                    int32_t ticks_br = response["encoders"]["back_right"];

                    wheels_[0].pos = (ticks_fl / COUNTS_PER_REV) * TWO_PI;
                    wheels_[1].pos = (ticks_fr / COUNTS_PER_REV) * TWO_PI;
                    wheels_[3].pos = (ticks_br / COUNTS_PER_REV) * TWO_PI;
                    wheels_[2].pos = wheels_[0].pos; 

                    if (!first_read && dt > 0.0) {
                        wheels_[0].vel = ((ticks_fl - last_ticks_fl) / COUNTS_PER_REV) * TWO_PI / dt; 
                        wheels_[1].vel = ((ticks_fr - last_ticks_fr) / COUNTS_PER_REV) * TWO_PI / dt; 
                        wheels_[3].vel = ((ticks_br - last_ticks_br) / COUNTS_PER_REV) * TWO_PI / dt; 
                        wheels_[2].vel = wheels_[0].vel;
                    }
                    last_ticks_fl = ticks_fl;
                    last_ticks_fr = ticks_fr;
                    last_ticks_br = ticks_br;
                    first_read = false;
                }
            } catch (json::parse_error &e) {
                RCLCPP_WARN(rclcpp::get_logger("CrobotHardware"), "Bad serial packet: %s", e.what());
                return hardware_interface::return_type::OK; 
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CrobotHardware::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        if (!serial_comm_.isConnected()) {
            RCLCPP_ERROR(rclcpp::get_logger("CrobotHardware"),
                         "Cannot write to hardware: not connected");
            return hardware_interface::return_type::ERROR;
        }

        json j;
        j["cmd"] = "write";

        const double RAD_TO_DEG = 180.0 / M_PI;

        j["ankles"]["front_left"] = (int)(120.0 + ankles_[0].cmd * RAD_TO_DEG / 0.75);
        j["ankles"]["front_right"] = (int)(60.0 + ankles_[1].cmd * RAD_TO_DEG / 0.75);
        j["ankles"]["back_left"] = (int)(60.0 + ankles_[2].cmd * RAD_TO_DEG / 0.75);
        j["ankles"]["back_right"] = (int)(120.0 + ankles_[3].cmd * RAD_TO_DEG / 0.75);

        const double MAX_WHEEL_SPEED = 0.8;  // m/s corresponding to full command (255)

        j["wheels"]["front_left"] = std::clamp((int)(wheels_[0].cmd / MAX_WHEEL_SPEED * 255.0), -255, 255);
        j["wheels"]["front_right"] = std::clamp((int)(wheels_[1].cmd / MAX_WHEEL_SPEED * 255.0), -255, 255);
        j["wheels"]["back_left"] = std::clamp((int)(wheels_[2].cmd / MAX_WHEEL_SPEED * 255.0), -255, 255);
        j["wheels"]["back_right"] = std::clamp((int)(wheels_[3].cmd / MAX_WHEEL_SPEED * 255.0), -255, 255);    
        
        j["sweeper"] = (int)(40.0 + sweeper_.cmd * RAD_TO_DEG);
        j["winch"] = (int)(winch_.cmd * 255.0);

        std::string j_str = j.dump() + "\n";

        // RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Sending JSON: %s", j_str.c_str());

        int bytesSent = serial_comm_.writeBytes(j_str.c_str(), j_str.size());

        if (bytesSent != (int)j_str.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("CrobotHardware"),
                         "Sent %d bytes, expected to send %zu bytes",
                         bytesSent, j_str.size());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    crobot_hardware::CrobotHardware, hardware_interface::SystemInterface
)