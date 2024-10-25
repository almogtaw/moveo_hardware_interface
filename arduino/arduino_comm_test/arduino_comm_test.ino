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
        Serial.println("Joint_1 curr_pos 0 curr_vel 0");
      } 
      else if (command.endsWith("Joint_2")) {
        Serial.println("Joint_2 curr_pos 0 curr_vel 0");
      } 
      else if (command.endsWith("Joint_3")) {
        Serial.println("Joint_3 curr_pos 0 curr_vel 0");
      } 
      else if (command.endsWith("Joint_4")) {
        Serial.println("Joint_4 curr_pos 0 curr_vel 0");
      } 
      else if (command.endsWith("Joint_5")) {
        Serial.println("Joint_5 curr_pos 0 curr_vel 0");
      } 
      else if (command.endsWith("Joint_6")) {
        Serial.println("Joint_6 curr_pos 0 curr_vel 0");
      } 
      else {
        Serial.println("Unknown joint requested");
      }
    } 
    else if (command.startsWith("Joint_")) {
      // Acknowledge position/velocity commands for each joint
      Serial.print("Command received: ");
      Serial.println(command);
    } 
    else {
      // Handle unexpected commands or noise
      Serial.println("Unknown command received");
    }
  }
  delay(10);  // Simulate processing time
}
