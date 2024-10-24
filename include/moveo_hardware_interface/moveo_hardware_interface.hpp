#ifndef MOVEO_HARDWARE_INTERFACE__MOVEO_HARDWARE_INTERFACE_HPP_
#define MOVEO_HARDWARE_INTERFACE__MOVEO_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visibility_control.h"
#include "arduino_comm.hpp"  // Include the Arduino communication code

namespace moveo_hardware_interface
{
class MoveoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info) override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  std::vector<double> position_commands_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  ArduinoComm arduino_;  // Arduino communication object
};

}  // namespace moveo_hardware_interface

#endif  // MOVEO_HARDWARE_INTERFACE__MOVEO_HARDWARE_INTERFACE_HPP_
