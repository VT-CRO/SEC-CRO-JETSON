#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "crobot_controller/crobot_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
    using ControllerReferenceMsg = crobot_controller::CrobotDriveController::ControllerReferenceMsg;

    void reset_controller_reference_msg(
        const std::shared_ptr<ControllerReferenceMsg> & msg,
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node
    )
    {
        msg->header.stamp = node->now();
        msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
        msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
        msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
        msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
        msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
        msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
    }
}

namespace crobot_controller
{

    CrobotDriveController::CrobotDriveController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn CrobotDriveController::on_init()
    {
        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
        }
        catch (const std::exception & e)
        {
            fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CrobotDriveController::on_configure(
        const rclcpp_lifecycle::State &
    )
    {
        params_ = param_listener_->get_params();

        auto prepare_command_interfaces_list = 
            [&command_joints = this->command_joint_names_](
                const std::size_t index, const std::string & command_joint_name
            )
        {
            command_joints[index] = command_joint_name;
        };

        command_joint_names_.resize(4);

        prepare_command_interfaces_list(
            FRONT_LEFT, params_.front_left_wheel_command_joint_name
        );
        prepare_command_interfaces_list(
            FRONT_RIGHT, params_.front_right_wheel_command_joint_name
        );
        prepare_command_interfaces_list(
            REAR_LEFT, params_.rear_left_wheel_command_joint_name
        );
        prepare_command_interfaces_list(
            REAR_RIGHT, params_.rear_right_wheel_command_joint_name
        );

        // configure deadwheels

        auto prepare_state_interfaces_list = 
            [&state_joints = this->state_joint_names_](
                const std::size_t index, const std::string & state_joint_name
            )
        {
            state_joints[index] = state_joint_name;
        };

        state_joint_names_.resize(3);

        prepare_state_interfaces_list(
            DEADWHEEL_X, params_.deadwheel_x_state_name
        );
        prepare_state_interfaces_list(
            DEADWHEEL_Y, params_.deadwheel_y_state_name
        );
        prepare_state_interfaces_list(
            DEADWHEEL_HEADING, params_.deadwheel_heading_state_name
        );

        auto subscribers_qos = rclcpp::SystemDefaultsQoS();
        subscribers_qos.keep_last(1);
        subscribers_qos.best_effort();

        ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
        ref_cmd_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
            "~/cmd_vel_stamped", subscribers_qos,
            std::bind(&CrobotDriveController::reference_callback, this, std::placeholders::_1));

        std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
        reset_controller_reference_msg(msg, get_node());
        input_ref_.writeFromNonRT(msg);
        
        try
        {
            odom_state_publisher_ = get_node()->create_publisher<OdomStateMsg>("~/odometry", rclcpp::SystemDefaultsQoS());
            rt_odom_state_publisher_ = std::make_unique<OdomStatePublisher>(odom_state_publisher_);
        } catch (const std::exception & e) {
            fprintf(
                stderr, "Exception thrown during publisher creation at configure state with message : %s \n",
                e.what()
            );
            return controller_interface::CallbackReturn::ERROR;
        }

        rt_odom_state_publisher_->lock();
        rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
        rt_odom_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
        rt_odom_state_publisher_->msg_.child_frame_id = params_.base_frame_id;
        rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;

        auto & covariance = rt_odom_state_publisher_->msg_.twist.covariance;
        constexpr size_t NUM_DIMENSION = 6;
        for (size_t index = 0; index < 6; ++index)
        {
            const size_t diagonal_index = NUM_DIMENSION * index + index;
            covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
            covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
        }
        rt_odom_state_publisher_->unlock();

        try
        {
            tf_odom_state_publisher_ = get_node()->create_publisher<TFStateMsg>("~/tf", rclcpp::SystemDefaultsQoS());
            rt_tf_odom_state_publisher_ = std::make_unique<TfStatePublisher>(tf_odom_state_publisher_);
        } catch (const std::exception & e) {
            fprintf(
                stderr, "Exception thrown during publisher creation at configure state with message : %s \n",
                e.what()
            );
            return controller_interface::CallbackReturn::ERROR;
        }

        rt_tf_odom_state_publisher_->lock();
        rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
        rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
        rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = params_.odom_frame_id;
        rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = params_.base_frame_id;
        rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0f;
        rt_tf_odom_state_publisher_->unlock();

        RCLCPP_INFO(get_node()->get_logger(), "CrobotDriveController configured successfully");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    void CrobotDriveController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
    {

        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
        {
            RCLCPP_WARN(
                get_node()->get_logger(),
                "Timestamp in header is missing, using current time as command timestamp."
            );
            msg->header.stamp = get_node()->now();
        }

        const auto age_of_last_command = get_node()->now() - msg->header.stamp;

        if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
        {
            input_ref_.writeFromNonRT(msg);
        } else {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Received message has timestamp %.10f older by %.10f than allowed timeout (%.4f).",
                rclcpp::Time(msg->header.stamp).seconds(), age_of_last_command.seconds(),
                ref_timeout_.seconds()
            );

            reset_controller_reference_msg(msg, get_node());
        }
    }

    controller_interface::InterfaceConfiguration CrobotDriveController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        command_interfaces_config.names.reserve(command_joint_names_.size());
        for (const auto & joint : command_joint_names_)
        {
            command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        }

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration CrobotDriveController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        state_interfaces_config.names.reserve(state_joint_names_.size());
        for (const auto & state_if : state_joint_names_)
        {
            state_interfaces_config.names.push_back(state_if);
        }

        return state_interfaces_config;
    }

    controller_interface::CallbackReturn CrobotDriveController::on_activate(
        const rclcpp_lifecycle::State &
    )
    {
        reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CrobotDriveController::on_deactivate(
        const rclcpp_lifecycle::State & // previous_state
    )
    {
        for (size_t i = 0; i < NUM_CMD_INTERFACES; ++i)
        {
            command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CrobotDriveController::update(
        const rclcpp::Time & time, const rclcpp::Duration & // period
    )
    {

        auto current_ref = *(input_ref_.readFromRT());

        const auto age_of_last_command = time - (current_ref)->header.stamp;

        if (age_of_last_command > ref_timeout_ && ref_timeout_!= rclcpp::Duration::from_seconds(0))
        {
            current_ref->twist.linear.x = 0.0;
            current_ref->twist.linear.y = 0.0;
            current_ref->twist.angular.z = 0.0;
        }

        // const double wheel_front_left_state_vel = state_interfaces_[FRONT_LEFT].get_value();
        // const double wheel_front_right_state_vel = state_interfaces_[FRONT_RIGHT].get_value();
        // const double wheel_rear_right_state_vel = state_interfaces_[REAR_RIGHT].get_value();
        // const double wheel_rear_left_state_vel = state_interfaces_[REAR_LEFT].get_value();

        double cmd_x = current_ref->twist.linear.x;
        double cmd_y = current_ref->twist.linear.y;
        double cmd_w = current_ref->twist.angular.z;
        
        // FORWARD KINEMATICS: Update Odometry
        // if (
        //     !std::isnan(wheel_front_left_state_vel) && !std::isnan(wheel_front_right_state_vel) &&
        //     !std::isnan(wheel_rear_left_state_vel) && !std::isnan(wheel_rear_right_state_vel)
        // )
        // {
            // odometry_.update(
            //      wheel_front_left_state_vel, wheel_rear_left_state_vel, wheel_rear_right_state_vel,
            //      wheel_front_right_state_vel, period.secconds()
            // );
        // }

        // INVERSE KINEMATICS: Compute wheel velocities
        if (
            !std::isnan(cmd_x) && !std::isnan(cmd_y) &&
            !std::isnan(cmd_w)
        )
        {
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, params_.kinematics.base_frame_offset.theta);

            tf2::Matrix3x3 rotation_from_base_to_center = tf2::Matrix3x3((quaternion));
            tf2::Vector3 velocity_in_base_frame_wrt_center_frame_ = rotation_from_base_to_center * tf2::Vector3(cmd_x, cmd_y, 0.0);
            tf2::Vector3 linear_trans_from_base_to_center = tf2::Vector3(
                params_.kinematics.base_frame_offset.x, params_.kinematics.base_frame_offset.y, 0.0);

            velocity_in_center_frame_linear_x_ =
                velocity_in_base_frame_wrt_center_frame_.x() +
                linear_trans_from_base_to_center.y() * cmd_w;
            velocity_in_center_frame_linear_y_ =
                velocity_in_base_frame_wrt_center_frame_.y() -
                linear_trans_from_base_to_center.x() * cmd_w;
            velocity_in_center_frame_angular_z_ = cmd_w;

            const double wheel_front_left_vel =
                1.0 / params_.kinematics.wheels_radius *
                (velocity_in_center_frame_linear_x_ - velocity_in_center_frame_linear_y_ -
                params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
                    velocity_in_center_frame_angular_z_);
            const double wheel_front_right_vel =
                1.0 / params_.kinematics.wheels_radius *
                (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ +
                params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
                    velocity_in_center_frame_angular_z_);
            const double wheel_rear_right_vel =
                1.0 / params_.kinematics.wheels_radius *
                (velocity_in_center_frame_linear_x_ - velocity_in_center_frame_linear_y_ +
                params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
                    velocity_in_center_frame_angular_z_);
            const double wheel_rear_left_vel =
                1.0 / params_.kinematics.wheels_radius *
                (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ -
                params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
                    velocity_in_center_frame_angular_z_);

            command_interfaces_[FRONT_LEFT].set_value(wheel_front_left_vel);
            command_interfaces_[FRONT_RIGHT].set_value(wheel_front_right_vel);
            command_interfaces_[REAR_LEFT].set_value(wheel_rear_left_vel);
            command_interfaces_[REAR_RIGHT].set_value(wheel_rear_right_vel);

            // ss << "Setting Wheel Speeds: " << wheel_front_left_vel << " " << wheel_front_right_vel << " " << wheel_rear_left_vel << " " << wheel_rear_right_vel;
            
        } else {
            command_interfaces_[FRONT_LEFT].set_value(0.0);
            command_interfaces_[FRONT_RIGHT].set_value(0.0);
            command_interfaces_[REAR_LEFT].set_value(0.0);
            command_interfaces_[REAR_RIGHT].set_value(0.0);
        }

        // update + publish tf

        // publish odometry

        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  crobot_controller::CrobotDriveController, controller_interface::ControllerInterface)