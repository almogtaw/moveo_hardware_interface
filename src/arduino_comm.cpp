#include "arduino_comm.hpp"
#include <iostream>

void ArduinoComm::connect()
{
  serial_port_.setPort("/dev/ttyUSB0");  // Adjust port as needed
  serial_port_.setBaudrate(115200);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial_port_.setTimeout(timeout);

  try
  {
    serial_port_.open();
  }
  catch (const serial::IOException & e)
  {
    std::cerr << "Unable to open port!" << std::endl;
  }

  if (serial_port_.isOpen())
  {
    std::cout << "Serial connection established!" << std::endl;
  }
  else
  {
    std::cerr << "Serial port not open!" << std::endl;
  }
}

void ArduinoComm::sendCommands(const std::vector<double> & positions, const std::vector<double> & velocities)
{
  std::string command = "SET ";
  for (size_t i = 0; i < positions.size(); ++i)
  {
    command += std::to_string(positions[i]) + " " + std::to_string(velocities[i]) + " ";
  }
  command += "\n";
  serial_port_.write(command);
}

void ArduinoComm::readStates(std::vector<double> & positions, std::vector<double> & velocities)
{
  std::string response = serial_port_.readline(1024, "\n");
  std::stringstream ss(response);
  for (size_t i = 0; i < positions.size(); ++i)
  {
    ss >> positions[i] >> velocities[i];
  }
}
