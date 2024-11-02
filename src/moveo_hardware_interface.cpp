#include "moveo_hardware_interface/moveo_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moveo_hardware_interface
{

  hardware_interface::CallbackReturn MoveoHardwareInterface::on_init(const hardware_interface::HardwareInfo &system_info)
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

    joints.clear(); // Ensure `joints` is empty before adding

    for (size_t i = 0; i < system_info.joints.size(); ++i)
    {
        const auto &joint_name = system_info.joints[i].name;
        joint_names_.push_back(joint_name);

        // Set specific steps-per-radian values based on joint index
        float steps_per_rad = 1500.0;  // Default value

        if (joint_name == "Joint_1")
            steps_per_rad = 7162;
        else if (joint_name == "Joint_2")
            steps_per_rad = 3756;
        else if (joint_name == "Joint_3")
            steps_per_rad = 697;
        else if (joint_name == "Joint_5")
            steps_per_rad = 2228;

        // Initialize each Joint object with name and specific steps_per_rad
        joints.emplace_back(Joint(joint_name, steps_per_rad));
    }

    RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"), "Moveo hardware interface initialized with joint names.");

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

  hardware_interface::return_type MoveoHardwareInterface::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    (void)stop_interfaces; // Mark as unused for no warnings - remove if used.

    // Iterate through the start_interfaces to determine control mode for each joint
    for (const auto &interface : start_interfaces)
    {
      for (size_t i = 0; i < joints.size(); ++i)
      {
        if (interface == joint_names_[i] + "/position")
        {
          joints[i].is_position_control = true; // Set to position control if position interface is found
          RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"),
                      "Setting joint %s to position control mode", joint_names_[i].c_str());
        }
        else if (interface == joint_names_[i] + "/velocity")
        {
          joints[i].is_position_control = false; // Set to velocity control if only velocity interface is found
          RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"),
                      "Setting joint %s to velocity control mode", joint_names_[i].c_str());
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MoveoHardwareInterface::perform_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    (void)start_interfaces; // Mark as unused for no warnings - remove if used.
    (void)stop_interfaces; // Mark as unused for no warnings - remove if used.
    // Additional hardware reconfiguration could be handled here if necessary.
    return hardware_interface::return_type::OK;
  }
  // -------------------------------------------------------------------------------------------------------------- //

  // ---------------------------------------------------- read ---------------------------------------------------- //
  hardware_interface::return_type MoveoHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &period)
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

    if (delta_seconds == 0) {
        RCLCPP_WARN(rclcpp::get_logger("MoveoHardwareInterface"), "Delta time is zero; skipping velocity calculation.");
        return hardware_interface::return_type::OK;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      double pos_prev = joints[i].pos;
      joints[i].steps = position_states_[i];
      joints[i].pos = joints[i].stepsToRadians(); // Convert steps to radians
      joints[i].vel = (joints[i].pos - pos_prev) / delta_seconds;

      // Update state vectors
      position_states_[i] = joints[i].pos;
      velocity_states_[i] = joints[i].vel;

      RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"),
                  "read - %s: position %f, velocity %f", joint_names_[i].c_str(), position_states_[i], velocity_states_[i]);
    }

    return hardware_interface::return_type::OK;
  }
  // -------------------------------------------------------------------------------------------------------------- //

  // ---------------------------------------------------- Write --------------------------------------------------- //
  hardware_interface::return_type MoveoHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    if (!arduino_.isConnected())
    {
      RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Arduino is disconnected.");
      return hardware_interface::return_type::ERROR;
    }

    // Convert position and velocity commands from radians to steps
    std::vector<int> step_positions(joint_names_.size());
    std::vector<int> step_velocities(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      // Convert each command to steps using the joint-specific conversion factor
      step_positions[i] = joints[i].radiansToSteps(position_commands_[i]);
      step_velocities[i] = joints[i].radiansToSteps(velocity_commands_[i]);
    }

    // Send converted positions and velocities to Arduino:
    for (size_t i = 0; i < joints.size(); ++i)
    {
      if (joints[i].is_position_control)
      {
        // Send position command (with optional velocity) for this joint:
        RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"),
                    "sending position command to arduino: %s: pos: %f, vel: %f",
                    joint_names_[i].c_str(), position_commands_[i], velocity_commands_[i]);

        if (arduino_.sendCommand(i + 1, step_positions[i], step_velocities[i]) != 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Error sending position commands to Arduino.");
          return hardware_interface::return_type::ERROR;
        }
      }
      else
      {
        // Send velocity command for this joint:
        RCLCPP_INFO(rclcpp::get_logger("MoveoHardwareInterface"),
                    "sending velocity command to arduino: %s: vel: %f",
                    joint_names_[i].c_str(), velocity_commands_[i]);

        if (arduino_.sendCommand(i + 1, step_velocities[i]) != 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("MoveoHardwareInterface"), "Error sending velocity commands to Arduino.");
          return hardware_interface::return_type::ERROR;
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

} // namespace moveo_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    moveo_hardware_interface::MoveoHardwareInterface, hardware_interface::SystemInterface)
