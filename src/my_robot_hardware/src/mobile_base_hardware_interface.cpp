#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include "my_robot_hardware/arduino_driver.hpp"

namespace mobile_base_hardware {

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;

    info_ = info;
    port_ = info_.hardware_parameters["serial_port"];

    if (port_.empty()) {
        RCLCPP_WARN(
            rclcpp::get_logger("MobileBaseHardwareInterface"),
            "No serial_port specified in hardware parameters. Trying default /dev/ttyUSB0"
        );
        port_ = "/dev/ttyUSB0";
    }

    driver_ = std::make_shared<ArduinoDriver>(port_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(
    const rclcpp_lifecycle::State &)
{
    if (driver_->init() != 0)
        return hardware_interface::CallbackReturn::ERROR;

    RCLCPP_INFO(get_logger(), "✅ Arduino serial driver configured on %s", port_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &)
{
    set_state("base_left_wheel_joint/velocity", 0.0);
    set_state("base_right_wheel_joint/velocity", 0.0);
    set_state("base_left_wheel_joint/position", 0.0);
    set_state("base_right_wheel_joint/position", 0.0);
    driver_->activate();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    driver_->deactivate();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardwareInterface::read(
    const rclcpp::Time &, const rclcpp::Duration &period)
{
    // --- Guard against invalid or zero period ---
    double dt = period.seconds();
    if (dt <= 0.0 || std::isnan(dt) || std::isinf(dt))
        dt = 0.02;  // default ~50 Hz

    // --- Read commanded velocities ---
    double left_vel = get_command("base_left_wheel_joint/velocity");
    double right_vel = get_command("base_right_wheel_joint/velocity");

    // --- Guard against NaNs ---
    if (std::isnan(left_vel) || std::isinf(left_vel)) left_vel = 0.0;
    if (std::isnan(right_vel) || std::isinf(right_vel)) right_vel = 0.0;

    // --- Integrate position ---
    double left_pos = get_state("base_left_wheel_joint/position") + left_vel * dt;
    double right_pos = get_state("base_right_wheel_joint/position") + right_vel * dt;

    if (std::isnan(left_pos) || std::isinf(left_pos)) left_pos = 0.0;
    if (std::isnan(right_pos) || std::isinf(right_pos)) right_pos = 0.0;

    // --- Update state interfaces ---
    set_state("base_left_wheel_joint/velocity", left_vel);
    set_state("base_right_wheel_joint/velocity", right_vel);
    set_state("base_left_wheel_joint/position", left_pos);
    set_state("base_right_wheel_joint/position", right_pos);

    // --- Optional debug log ---
    if (std::fabs(left_vel) > 1e-3 || std::fabs(right_vel) > 1e-3)
    {
        RCLCPP_DEBUG(get_logger(), "Left=%.2f rad/s | Right=%.2f rad/s", left_vel, right_vel);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    double left_cmd = get_command("base_left_wheel_joint/velocity");
    double right_cmd = get_command("base_right_wheel_joint/velocity");

    // --- Sanitize commands ---
    if (std::isnan(left_cmd) || std::isinf(left_cmd)) left_cmd = 0.0;
    if (std::isnan(right_cmd) || std::isinf(right_cmd)) right_cmd = 0.0;

    // Invert direction if needed
    right_cmd = -right_cmd;

    driver_->setTargetVelocity(left_cmd, right_cmd);
    return hardware_interface::return_type::OK;
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)

