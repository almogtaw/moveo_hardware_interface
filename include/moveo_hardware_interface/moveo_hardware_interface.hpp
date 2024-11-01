#ifndef MOVEO_HARDWARE_INTERFACE__MOVEO_HARDWARE_INTERFACE_HPP_
#define MOVEO_HARDWARE_INTERFACE__MOVEO_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "moveo_hardware_interface/visibility_control.h"
#include "moveo_hardware_interface/arduino_comm.hpp"
#include "moveo_hardware_interface/joint.hpp"

namespace moveo_hardware_interface
{
class MoveoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MoveoHardwareInterface);

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn 
  on_init(const hardware_interface::HardwareInfo & system_info) override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn 
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn 
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> 
  export_state_interfaces() override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> 
  export_command_interfaces() override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;


  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type 
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MOVEO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type 
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<double> position_commands_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  ArduinoComm arduino_;  // Arduino communication object
  std::vector<Joint> joints;
};

}  // namespace moveo_hardware_interface

#endif  // MOVEO_HARDWARE_INTERFACE__MOVEO_HARDWARE_INTERFACE_HPP_
