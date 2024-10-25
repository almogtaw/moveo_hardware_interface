#include "moveo_hardware_interface/arduino_comm.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
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
  try {
      serial_conn_.Open("/dev/ttyUSB0");
      serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
      serial_conn_.FlushIOBuffers();  // Clear old data
      std::cout << "Serial connection established!" << std::endl;
      return 0;
  } catch (const LibSerial::OpenFailed&) {
      std::cerr << "Failed to open serial device " << std::endl;
      return -1;
  }
}

void ArduinoComm::disconnect() {
  if (serial_conn_.IsOpen()) {
    serial_conn_.Close();
  }
}

bool ArduinoComm::isConnected() const {
  return serial_conn_.IsOpen();
}

// Send command to Arduino and wait for response
std::string ArduinoComm::send_msg(const std::string &msg_to_send) 
{
  serial_conn_.FlushIOBuffers();  // Clear old data
  serial_conn_.Write(msg_to_send);

  std::string response = "";
  try 
  {
    serial_conn_.ReadLine(response, '\n', timeout_ms_);
  } 
  catch (const LibSerial::ReadTimeout&) {
    std::cerr << "Read operation timed out." << std::endl;
  }

  // std::cout << "[send_msg] response:" << response.str() << std::endl;

  return response;
}

// Send position and velocity commands
int ArduinoComm::sendCommands(const std::vector<double>& positions, const std::vector<double>& velocities) 
{
  if (!isConnected()) {
    std::cerr << "Attempted to send commands, but not connected to Arduino." << std::endl;
  return -1;  // Error code for not connected
  }

  for (size_t i = 0; i < positions.size(); ++i) 
  {
    // Create command for each joint
    std::stringstream command;
    command << "Joint_" << (i + 1) << " position " << positions[i]
            << " velocity " << velocities[i] << "\n";

    std::cout << "[sendCommands] sending command: " << command.str() << std::endl;

    // Send command and wait for response
    auto response = send_msg(command.str());
    if (response.empty()) {
      std::cerr << "No response from Arduino after sending command for Joint_" << (i + 1) << "." << std::endl;
      return -1;  // Exit if any command fails
    }
    std::cout << "[sendCommands] response:" << response << std::endl;
  }
  return 0;  // Success code
}

int ArduinoComm::readStates(std::vector<double>& positions, std::vector<double>& velocities) 
{
  if (!isConnected()) {
    std::cerr << "Attempted to read states, but not connected to Arduino." << std::endl;
    return -1;
  }

  // Request joint states from Arduino
  auto response = send_msg("READ_STATES");
  if (response.empty()) {
    std::cerr << "No response from Arduino for state request." << std::endl;
    return -1;
  }

  std::stringstream ss(response);
  std::string joint_name, curr_pos_str, curr_vel_str;
  double curr_pos, curr_vel;

  for (size_t i = 0; i < positions.size(); ++i) {
    ss >> joint_name >> curr_pos_str >> curr_pos >> curr_vel_str >> curr_vel;
    if (curr_pos_str == "curr_pos" && curr_vel_str == "curr_vel") {
      positions[i] = curr_pos;
      velocities[i] = curr_vel;
    } else {
      std::cerr << "Unexpected format in Arduino response: " << response << std::endl;
      return -1;
    }
  }
  return 0;
}
