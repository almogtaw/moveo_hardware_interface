#include "arduino_comm.hpp"
#include <iostream>
#include <sstream>

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
  serial_port_.setPort("/dev/ttyUSB0");  // Adjust port as needed
  serial_port_.setBaudrate(57600);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial_port_.setTimeout(timeout);

  try
  {
    serial_port_.open();
  }
  catch (const serial::IOException & e)
  {
    std::cerr << "Unable to open port: " << e.what() << std::endl;
    return -1;  // Error code for connection failure
  }

  if (isConnected())
  {
    std::cout << "Serial connection established!" << std::endl;
    return 0;  // Success code
  }
  else
  {
    std::cerr << "Serial port not open!" << std::endl;
    return -1;  // Error code for connection failure
  }
}

void ArduinoComm::disconnect()
{
  if (isConnected())
  {
    serial_port_.close();
    std::cout << "Serial connection closed." << std::endl;
  }
}

bool ArduinoComm::isConnected() const
{
  return serial_port_.isOpen();
}

int ArduinoComm::sendCommands(const std::vector<double> & positions, const std::vector<double> & velocities)
{
  if (!isConnected())
  {
    std::cerr << "Attempted to send commands, but not connected to Arduino." << std::endl;
    return -1;  // Error code for not connected
  }

  std::stringstream command;
  for (size_t i = 0; i < 6; ++i)
  {
    command << "Joint_" << (i + 1) << " position " << positions[i] 
            << " velocity " << velocities[i] << "\n";
  }

  try
  {
    serial_port_.write(command.str());
  }
  catch (const serial::IOException & e)
  {
    std::cerr << "Error sending commands: " << e.what() << std::endl;
    return -1;  // Error code for write failure
  }

  return 0;  // Success code
}

int ArduinoComm::readStates(std::vector<double> & positions, std::vector<double> & velocities)
{
  if (!isConnected())
  {
    std::cerr << "Attempted to read states, but not connected to Arduino." << std::endl;
    return -1;  // Error code for not connected
  }

  // Send the read request to Arduino
  try
  {
    serial_port_.write("READ_STATES\n");
  }
  catch (const serial::IOException & e)
  {
    std::cerr << "Error sending read request: " << e.what() << std::endl;
    return -1;  // Error code for write failure
  }

  // Read the response for each joint
  try
  {
    for (size_t i = 0; i < 6; ++i)
    {
      std::string response = serial_port_.readline(1024, "\n");
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
  catch (const serial::IOException & e)
  {
    std::cerr << "Error reading states: " << e.what() << std::endl;
    return -1;  // Error code for read failure
  }

  return 0;  // Success code
}
