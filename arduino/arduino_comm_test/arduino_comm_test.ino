// Define the number of joints
#define NUM_JOINTS 6

// Arrays to store the last position and velocity commands for each joint
float joint_positions[NUM_JOINTS] = {0.9, 0.9, 0.9, 0, 0, 0};
float joint_velocities[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};  // Initialize velocities to zero

void setup() {
  Serial.begin(57600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Check if the command is to read a specific joint's state
    if (command.startsWith("READ_STATE")) {
      // Determine which joint's state is requested
      if (command.endsWith("Joint_1")) {
        Serial.print("Joint_1 curr_pos ");
        Serial.print(joint_positions[0]);
        Serial.print(" curr_vel ");
        Serial.println(joint_velocities[0]);
      } 
      else if (command.endsWith("Joint_2")) {
        Serial.print("Joint_2 curr_pos ");
        Serial.print(joint_positions[1]);
        Serial.print(" curr_vel ");
        Serial.println(joint_velocities[1]);
      } 
      else if (command.endsWith("Joint_3")) {
        Serial.print("Joint_3 curr_pos ");
        Serial.print(joint_positions[2]);
        Serial.print(" curr_vel ");
        Serial.println(joint_velocities[2]);
      } 
      else if (command.endsWith("Joint_4")) {
        Serial.print("Joint_4 curr_pos ");
        Serial.print(joint_positions[3]);
        Serial.print(" curr_vel ");
        Serial.println(joint_velocities[3]);
      } 
      else if (command.endsWith("Joint_5")) {
        Serial.print("Joint_5 curr_pos ");
        Serial.print(joint_positions[4]);
        Serial.print(" curr_vel ");
        Serial.println(joint_velocities[4]);
      } 
      else if (command.endsWith("Joint_6")) {
        Serial.print("Joint_6 curr_pos ");
        Serial.print(joint_positions[5]);
        Serial.print(" curr_vel ");
        Serial.println(joint_velocities[5]);
      } 
      else {
        Serial.println("Unknown joint requested");
      }
    } 
    else if (command.startsWith("Joint_")) {
      // Extract the joint index from the command
      int joint_index = command.substring(6, 7).toInt() - 1;
      
      // Check if the joint index is valid
      if (joint_index >= 0 && joint_index < NUM_JOINTS) {
        // Update the joint's position if "position" is specified in the command
        int pos_index = command.indexOf("position");
        if (pos_index != -1) {
          String pos_value = command.substring(pos_index + 9);
          joint_positions[joint_index] = pos_value.toFloat();  // Convert to float for precision
        }
        
        // Update the joint's velocity if "velocity" is specified in the command
        int vel_index = command.indexOf("velocity");
        if (vel_index != -1) {
          String vel_value = command.substring(vel_index + 9);
          joint_velocities[joint_index] = vel_value.toFloat();  // Convert to float for precision
        }
        
        // Acknowledge the position and velocity command
        Serial.print("Command received for Joint_");
        Serial.print(joint_index + 1);
        Serial.print(": position set to ");
        Serial.print(joint_positions[joint_index]);
        Serial.print(", velocity set to ");
        Serial.println(joint_velocities[joint_index]);
      } else {
        Serial.println("Invalid joint command received");
      }
    } 
    else {
      // Handle unexpected commands or noise
      Serial.println("Unknown command received");
    }
  }
  delay(10);  // Simulate processing time
}
