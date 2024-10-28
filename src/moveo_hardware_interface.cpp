#include "moveo_hardware_interface/moveo_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moveo_hardware_interface
{

hardware_interface::CallbackReturn MoveoHardwareInterface::on_init(const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Populate joint names and initialize state and command vectors
  joint_names_.clear();
  position_commands_.resize(system_info.joints.size(), 0.0);
  velocity_commands_.resize(system_info.joints.size(), 0.0);
  position_states_.resize(system_info.joints.size(), 0.0);
  velocity_states_.resize(system_info.joints.size(), 0.0);

  for (const auto & joint : system_info.joints)
  {
    joint_names_.push_back(joint.name);
  }

  RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"), "Moveo hardware interface initialized with joint names.");
  // for (size_t i = 0; i < joint_names_.size(); ++i)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"), "Joint name: %s", joint_names_[i].c_str());
  // }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoveoHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (arduino_.connect() != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Failed to connect to Arduino.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoveoHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  arduino_.disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoveoHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    // add stepper motors enable
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoveoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    // add stepper motors disable
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MoveoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MoveoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type MoveoHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!arduino_.isConnected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Arduino is disconnected.");
    return hardware_interface::return_type::ERROR;
  }

  // Request joint states from Arduino
  if (arduino_.readStates(position_states_, velocity_states_) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Error reading states from Arduino.");
    return hardware_interface::return_type::ERROR;
  }

  double delta_seconds = period.seconds();

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    double pos_prev = joints[i].pos;
    joints[i].steps = position_states_[i];
    joints[i].pos = joints[i].stepsToRadians();
    joints[i].vel = (joints[i].pos - pos_prev) / delta_seconds;

    velocity_states_[i] = joints[i].vel;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoveoHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!arduino_.isConnected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Arduino is disconnected.");
    return hardware_interface::return_type::ERROR;
  }

  // Send position and velocity commands to Arduino
  if (arduino_.sendCommands(position_commands_, velocity_commands_) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Error sending commands to Arduino.");
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"), 
                "write - %s: position %f, velocity %f", joint_names_[i].c_str(), position_commands_[i], velocity_commands_[i]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace moveo_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  moveo_hardware_interface::MoveoHardwareInterface, hardware_interface::SystemInterface)
