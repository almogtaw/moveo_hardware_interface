#include "moveo_hardware_interface/arduino_comm.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <thread>
#include <chrono>

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
      serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      serial_conn_.FlushIOBuffers();  // Clear old data
      std::this_thread::sleep_for(std::chrono::seconds(2));
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
    // Convert radians to steps
    long position_steps = static_cast<long>(positions[i] * steps_per_rad[i]);
    long velocity_steps = static_cast<long>(velocities[i] * steps_per_rad[i]);

    // Create command for each joint
    std::stringstream command;
    command << "Joint_" << (i + 1) << " position " << position_steps
            << " velocity " << velocity_steps << "\n";

    // std::cout << "[sendCommands] sending command: " << command.str() << std::endl;

    // Send command and wait for response
    auto response = send_msg(command.str());
    if (response.empty()) {
      std::cerr << "No response from Arduino after sending command for Joint_" << (i + 1) << "." << std::endl;
      return -1;  // Exit if any command fails
    }
    // std::cout << "[sendCommands] response:" << response << std::endl;
  }
  return 0;  // Success code
}

int ArduinoComm::readStates(std::vector<double>& positions, std::vector<double>& velocities) 
{
  if (!isConnected()) {
    std::cerr << "Attempted to read states, but not connected to Arduino." << std::endl;
    return -1;
  }

  for (size_t i = 0; i < positions.size(); ++i) 
  {
    // Send a request for each joint state individually
    std::stringstream command;
    command << "READ_STATE Joint_" << (i + 1) << "\n";
    
    // Send the command to Arduino and wait for the response
    auto response = send_msg(command.str());
    if (response.empty()) 
    {
      std::cerr << "No response from Arduino for Joint_" << (i + 1) << " state request." << std::endl;
      return -1;  // Exit if any command fails
    }

    // std::cout << "[readStates] received response for Joint_" << (i + 1) << ": " << response << std::endl;

    // Parse the response for current position and velocity
    
    std::stringstream ss(response);
    std::string joint_name, curr_pos_str, curr_vel_str;
    long curr_pos_steps, curr_vel_steps;

    ss >> joint_name >> curr_pos_str >> curr_pos_steps >> curr_vel_str >> curr_vel_steps;
    if (curr_pos_str == "curr_pos" && curr_vel_str == "curr_vel") 
    {
      positions[i] = static_cast<double>(curr_pos_steps) / steps_per_rad[i];
      velocities[i] = static_cast<double>(curr_vel_steps) / steps_per_rad[i];
    } 
    else 
    {
      std::cerr << "Unexpected format in Arduino response: " << response << std::endl;
      return -1;  // Error code for unexpected format
    }
  }
  return 0;  // Success code
}

