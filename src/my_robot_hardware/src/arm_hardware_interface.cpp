// #include "my_robot_hardware/arm_hardware_interface.hpp"
// #include "my_robot_hardware/arduino_driver.hpp"

// namespace arm_hardware
// {
// hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
//     const hardware_interface::HardwareInfo & info)
// {
//     if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
//         return hardware_interface::CallbackReturn::ERROR;

//     info_ = info;
//     port_ = info_.hardware_parameters["serial_port"];

//     if (port_.empty()) {
//         RCLCPP_WARN(
//             rclcpp::get_logger("MobileBaseHardwareInterface"),
//             "No serial_port specified in hardware parameters. Trying default /dev/ttyUSB0"
//         );
//         port_ = "/dev/ttyUSB0";
//     }

//     driver_ = std::make_shared<ArduinoDriver>(port_);
//     return hardware_interface::CallbackReturn::SUCCESS;
// }
