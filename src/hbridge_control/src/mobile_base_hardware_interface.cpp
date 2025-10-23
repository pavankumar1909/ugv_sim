#include "mobile_base_hardware_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>

namespace mobile_base_hardware {

using CallbackReturn = hardware_interface::CallbackReturn;

CallbackReturn MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Read serial port from hardware parameters
    if (info.hardware_parameters.find("serial_port") != info.hardware_parameters.end()) {
        port_ = info.hardware_parameters.at("serial_port");
    } else {
        RCLCPP_WARN(get_logger(), "No 'serial_port' parameter, using default /dev/ttyUSB0");
        port_ = "/dev/ttyUSB0";
    }

    max_speed_ = 255;  // set maximum speed for your motors
    left_velocity_command_ = 0.0;
    right_velocity_command_ = 0.0;
    left_velocity_state_ = 0.0;
    right_velocity_state_ = 0.0;
    return CallbackReturn::SUCCESS;
}

CallbackReturn MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    serial_port_ = open(port_.c_str(), O_RDWR);
    if (serial_port_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", port_.c_str());
        return CallbackReturn::ERROR;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_port_, &tty) != 0) {
        RCLCPP_ERROR(get_logger(), "Error getting serial port attributes");
        close(serial_port_);
        return CallbackReturn::ERROR;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON & ~ECHO & ~ECHOE & ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(get_logger(), "Error setting serial port attributes");
        close(serial_port_);
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Serial port configured successfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    left_velocity_command_ = 0.0;
    right_velocity_command_ = 0.0;
    left_velocity_state_ = 0.0;
    right_velocity_state_ = 0.0;

    set_state("base_left_wheel_joint/velocity", 0.0);
    set_state("base_right_wheel_joint/velocity", 0.0);
    set_state("base_left_wheel_joint/position", 0.0);
    set_state("base_right_wheel_joint/position", 0.0);

    return CallbackReturn::SUCCESS;
}

CallbackReturn MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    if (serial_port_ >= 0) close(serial_port_);
    serial_port_ = -1;
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardwareInterface::read(
    const rclcpp::Time &, const rclcpp::Duration & period)
{
    // Simple simulation if no encoder: integrate velocity over time
    left_velocity_state_ += left_velocity_command_ * period.seconds();
    right_velocity_state_ += right_velocity_command_ * period.seconds();

    // Ensure no NaN values
    left_velocity_state_ = std::isfinite(left_velocity_state_) ? left_velocity_state_ : 0.0;
    right_velocity_state_ = std::isfinite(right_velocity_state_) ? right_velocity_state_ : 0.0;

    set_state("base_left_wheel_joint/velocity", left_velocity_command_);
    set_state("base_right_wheel_joint/velocity", right_velocity_command_);
    set_state("base_left_wheel_joint/position", left_velocity_state_);
    set_state("base_right_wheel_joint/position", right_velocity_state_);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    if (serial_port_ < 0) return hardware_interface::return_type::ERROR;

    // Compute clamped and rounded speeds
    double left_cmd = left_velocity_command_ * max_speed_;
    left_cmd = std::clamp(left_cmd, -static_cast<double>(max_speed_), static_cast<double>(max_speed_));
    int left_speed = static_cast<int>(std::round(left_cmd));

    double right_cmd = right_velocity_command_ * max_speed_;
    right_cmd = std::clamp(right_cmd, -static_cast<double>(max_speed_), static_cast<double>(max_speed_));
    int right_speed = static_cast<int>(std::round(right_cmd));

    if (!sendSerialCommand(left_speed, right_speed)) {
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


bool MobileBaseHardwareInterface::sendSerialCommand(int speed_left, int speed_right)
{
    if (serial_port_ < 0) return false;

    auto buildCommand = [](int speed) {
        if (speed > 0) return "F" + std::to_string(speed) + "\n";
        if (speed < 0) return "B" + std::to_string(-speed) + "\n";
        return std::string("S0\n");
    };

    std::string cmd = buildCommand(speed_left) + buildCommand(speed_right);
    ssize_t written = ::write(serial_port_, cmd.c_str(), cmd.size());
    return written == static_cast<ssize_t>(cmd.size());
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
