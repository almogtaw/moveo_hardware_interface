#include "moveo_hardware_interface/arduino_comm.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
  ArduinoComm arduino;

  // Step 1: Connect to the Arduino
  if (arduino.connect() != 0) {
    std::cerr << "Failed to connect to Arduino." << std::endl;
    return -1;
  }
  std::cout << "Successfully connected to Arduino!" << std::endl;

  // std::this_thread::sleep_for(std::chrono::seconds(2));

  // Step 2: Send mock position and velocity commands
  std::vector<double> positions = {100, 110, 120, 130, 140, 150};
  std::vector<double> velocities = {50, 55, 60, 65, 70, 75};
  if (arduino.sendCommands(positions, velocities) != 0) {
    std::cerr << "Failed to send commands to Arduino." << std::endl;
    return -1;
  }
  std::cout << "Commands sent successfully!" << std::endl;

  if (true)
  {
    // Step 3: Request and read states from the Arduino
    std::vector<double> read_positions(6);
    std::vector<double> read_velocities(6);
    if (arduino.readStates(read_positions, read_velocities) != 0) {
      std::cerr << "Failed to read states from Arduino." << std::endl;
      return -1;
    }

    // Step 4: Print received states
    for (size_t i = 0; i < read_positions.size(); ++i) {
      std::cout << "Joint_" << (i + 1) << " - Position: " << read_positions[i] 
                << ", Velocity: " << read_velocities[i] << std::endl;
    }
  }
  // Step 5: Disconnect
  arduino.disconnect();
  std::cout << "Disconnected from Arduino." << std::endl;

  return 0;
}
