#ifndef ARDUINO_COMM_HPP_
#define ARDUINO_COMM_HPP_

#include <vector>
#include <string>
#include <libserial/SerialPort.h>  // Serial communication library

class ArduinoComm
{
public:
  ArduinoComm();
  ~ArduinoComm();

  int connect();  // Establish the serial connection
  void disconnect();  // Close the serial connection
  bool isConnected() const;
  int sendCommands(const std::vector<double> & positions, const std::vector<double> & velocities);  // Send command data
  int readStates(std::vector<double> & positions, std::vector<double> & velocities);  // Read state data

private:
  LibSerial::SerialPort serial_stream_;
};

#endif  // ARDUINO_COMM_HPP_
