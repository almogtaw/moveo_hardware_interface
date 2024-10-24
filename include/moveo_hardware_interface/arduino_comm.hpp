#ifndef ARDUINO_COMM_HPP_
#define ARDUINO_COMM_HPP_

#include <vector>
#include <string>
#include <serial/serial.h>  // Serial communication library

class ArduinoComm
{
public:
  ArduinoComm() = default;

  void connect();
  void sendCommands(const std::vector<double> & positions, const std::vector<double> & velocities);
  void readStates(std::vector<double> & positions, std::vector<double> & velocities);

private:
  serial::Serial serial_port_;
};

#endif  // ARDUINO_COMM_HPP_
