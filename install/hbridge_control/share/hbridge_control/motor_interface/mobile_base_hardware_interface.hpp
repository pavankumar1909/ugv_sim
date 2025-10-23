#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace mobile_base_hardware {

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int serial_port_{-1};
  std::string port_;
  int max_speed_{255};

  double left_velocity_command_{0.0};
  double right_velocity_command_{0.0};

  double left_velocity_state_{0.0};
  double right_velocity_state_{0.0};

  bool sendSerialCommand(int speed_left, int speed_right);
};

} // namespace mobile_base_hardware
