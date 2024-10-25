#include "moveo_hardware_interface/arduino_comm.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>

ArduinoComm::ArduinoComm() {}

ArduinoComm::~ArduinoComm()
{
  if (isConnected())
  {
    disconnect();
  }
}

int ArduinoComm::connect()
{
  serial_stream_.Open("/dev/ttyUSB0");  // Set the correct device path
  serial_stream_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
  serial_stream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serial_stream_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  serial_stream_.SetParity(LibSerial::Parity::PARITY_NONE);
  serial_stream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

  if (!serial_stream_.IsOpen())
  {
    std::cerr << "Unable to open serial port." << std::endl;
    return -1;  // Error code for connection failure
  }

  std::cout << "Serial connection established!" << std::endl;
  return 0;  // Success code
}

void ArduinoComm::disconnect()
{
  if (serial_stream_.IsOpen())
  {
    serial_stream_.Close();
    std::cout << "Serial connection closed." << std::endl;
  }
}

int ArduinoComm::sendCommands(const std::vector<double>& positions, const std::vector<double>& velocities)
{
  if (!isConnected())
  {
    std::cerr << "Attempted to send commands, but not connected to Arduino." << std::endl;
    return -1;  // Error code for not connected
  }

  std::stringstream command;
  for (size_t i = 0; i < positions.size(); ++i)
  {
    command << "Joint_" << (i + 1) << " position " << positions[i] 
            << " velocity " << velocities[i] << "\n";
  }

  try
  {
    serial_stream_ << command.str();
  }
  catch (const std::runtime_error& e)
  {
    std::cerr << "Error sending commands: " << e.what() << std::endl;
    return -1;  // Error code for write failure
  }

  return 0;  // Success code
}

int ArduinoComm::readStates(std::vector<double>& positions, std::vector<double>& velocities)
{
  if (!isConnected())
  {
    std::cerr << "Attempted to read states, but not connected to Arduino." << std::endl;
    return -1;  // Error code for not connected
  }

  try
  {
    // Send the read request to Arduino
    serial_stream_ << "READ_STATES\n";

    // Read the response for each joint
    for (size_t i = 0; i < positions.size(); ++i)
    {
      std::string response;
      std::getline(serial_stream_, response);

      std::stringstream ss(response);
      std::string joint_name, curr_pos_str, curr_vel_str;
      double curr_pos, curr_vel;

      ss >> joint_name >> curr_pos_str >> curr_pos >> curr_vel_str >> curr_vel;

      // Ensure we have expected keywords
      if (curr_pos_str == "curr_pos" && curr_vel_str == "curr_vel")
      {
        positions[i] = curr_pos;
        velocities[i] = curr_vel;
      }
      else
      {
        std::cerr << "Unexpected response format: " << response << std::endl;
        return -1;  // Error code for incorrect format
      }
    }
  }
  catch (const std::runtime_error& e)
  {
    std::cerr << "Error reading states: " << e.what() << std::endl;
    return -1;  // Error code for read failure
  }

  return 0;  // Success code
}

bool ArduinoComm::isConnected() const
{
  return serial_stream_.IsOpen();
}
