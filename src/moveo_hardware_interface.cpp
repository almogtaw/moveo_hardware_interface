#include "moveo_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moveo_hardware_interface
{

hardware_interface::return_type MoveoHardwareInterface::on_init(const hardware_interface::HardwareInfo & system_info)
{
  position_commands_.resize(6, 0.0);
  velocity_commands_.resize(6, 0.0);
  position_states_.resize(6, 0.0);
  velocity_states_.resize(6, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"), "Moveo hardware interface initialized.");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoveoHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  arduino_.connect();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoveoHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  arduino_.disconnect();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoveoHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoveoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MoveoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < 6; ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "joint_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "joint_" + std::to_string(i), hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MoveoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < 6; ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "joint_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "joint_" + std::to_string(i), hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type MoveoHardwareInterface::read()
{
  // Receive state data from Arduino
  if (arduino_.readStates(position_states_, velocity_states_) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Error reading from Arduino.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoveoHardwareInterface::write()
{
  // Send commands to Arduino
  if (arduino_.sendCommands(position_commands_, velocity_commands_) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Error writing to Arduino.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace moveo_hardware_interface
