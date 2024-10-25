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
  command << "SET ";
  for (size_t i = 0; i < positions.size(); ++i)
  {
    command << positions[i] << " " << velocities[i] << " ";
  }
  command << "\n";

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

  try
  {
    std::string response = serial_port_.readline(1024, "\n");
    std::stringstream ss(response);
    for (size_t i = 0; i < positions.size(); ++i)
    {
      ss >> positions[i] >> velocities[i];
    }
  }
  catch (const serial::IOException & e)
  {
    std::cerr << "Error reading states: " << e.what() << std::endl;
    return -1;  // Error code for read failure
  }

  return 0;  // Success code
}

