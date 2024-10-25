void setup() {
  Serial.begin(57600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); 

    if (command.startsWith("READ_STATES")) 
    {
      // Serial.println("Command received: READ_STATES");
      // Send current positions and velocities for each joint
      Serial.println("Joint_1 curr_pos 100 curr_vel 50");
      Serial.println("Joint_2 curr_pos 110 curr_vel 55");
      Serial.println("Joint_3 curr_pos 120 curr_vel 60");
      Serial.println("Joint_4 curr_pos 130 curr_vel 65");
      Serial.println("Joint_5 curr_pos 140 curr_vel 70");
      Serial.println("Joint_6 curr_pos 150 curr_vel 75");
      // Serial.println("END");  // Use "END" to mark the end of the response
    } 
    else if (command.startsWith("Joint_")) 
    {
      // Acknowledge each joint command received
      Serial.print("Command received: ");
      Serial.println(command);
    } 
    else 
    {
      // Handle unexpected commands or noise
      Serial.println("Unknown command received");
    }
  }
  delay(10);  // Simulate processing time
}
