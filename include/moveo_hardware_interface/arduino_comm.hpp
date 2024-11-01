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

  std::string send_msg(const std::string &msg_to_send);

  // Sends position and velocity commands for each joint
  int sendCommand(const int joint_num, const long position, const long velocity); // send position + velocity commnad
  int sendCommand(const int joint_num, const long velocity); // send velocity commnad

  // Reads the current position and velocity of each joint
  int readStates(std::vector<double>& positions, std::vector<double>& velocities);

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_ = 1000;
};

#endif  // ARDUINO_COMM_HPP_
