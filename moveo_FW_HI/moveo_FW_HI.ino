#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// Joint 1
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

// Joint 2
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

// Joint 3
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

// Joint 4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// Joint 5 
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

// Max velocities - empirical
#define joint_1_max_vel    0
#define joint_2_max_vel    0
#define joint_3_max_vel    4000
#define joint_4_max_vel    0   
#define joint_5_max_vel    5000

// Max accelerations - empirical
#define joint_1_max_accel    0
#define joint_2_max_accel    0
#define joint_3_max_accel    10000
#define joint_4_max_accel    0   
#define joint_5_max_accel    5000

struct joint_parameters
{
  long min_travel;       // [steps]
  long max_travel;       // [steps]
  float min_vel;        // [steps/sec]
  float max_vel;        // [steps/sec]
  float default_vel;    // [steps/sec]
  float max_accel;      // [steps/sec^2]
  float default_accel;  // [steps/sec^2]

  // Constructor to initialize the values
  joint_parameters(long mt = 0, long mxt = 0, float miv = 0.0, float mav = 0.0, float dv = 0.0, float maa = 0.0, float da = 0.0)
    : min_travel(mt), max_travel(mxt), min_vel(miv), max_vel(mav), default_vel(dv), max_accel(maa), default_accel(da) {}
};

// Initialize each joint with tested parameters:
joint_parameters joint1_param(-25000, 3000, -40000.0, 40000.0, 40000.0, 40000.0, 40000.0);
joint_parameters joint2_param(-6100, 7000, -60000.0, 60000.0, 30000.0, 50000.0, 50000.0);
joint_parameters joint3_param(-1500, 1500, -1000.0, 1000.0, 700.0, 10000.0, 10000.0);
joint_parameters joint4_param;  // inactive joint - all values initialize to zero in the constructor
joint_parameters joint5_param(-4200, 3500, -4000.0, 4000.0, 2500.0, 5000.0, 5000.0);

// Create an array of pointers to the instances
joint_parameters* joints_params[] = {&joint1_param, &joint2_param, &joint3_param, &joint4_param, &joint5_param};

AccelStepper joint1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper joint2(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint3(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint4(AccelStepper::DRIVER, E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint5(AccelStepper::DRIVER, E0_STEP_PIN, E0_DIR_PIN);

Servo gripper;
MultiStepper steppers;

AccelStepper* joints[] = {&joint1, &joint2, &joint3, &joint4, &joint5};

// Define the number of joints
#define NUM_JOINTS 5

// Initialize each joint's last position and velocity
long joint_positions[NUM_JOINTS] = {0, 0, 0, 0, 0};
long joint_velocities[NUM_JOINTS] = {0, 0, 0, 0, 0};

// Control mode flag for all joints
bool is_position_control = true;

void setup() 
{
  Serial.begin(115200); // Start serial communication
  pinMode(13, OUTPUT);

  // Configure each stepper enable pin
  joint1.setEnablePin(X_ENABLE_PIN);
  joint2.setEnablePin(Z_ENABLE_PIN);
  joint3.setEnablePin(Y_ENABLE_PIN);
  joint4.setEnablePin(E1_ENABLE_PIN);
  joint5.setEnablePin(E0_ENABLE_PIN);

  // Configure each stepper
  for (int i = 0; i < NUM_JOINTS; ++i) 
  {
    joints[i]->setPinsInverted(false, false, true); // Configure each stepper enable pin inverted
    joints[i]->enableOutputs(); // enable outputs
    joints[i]->setMaxSpeed(joints_params[i]->max_vel);  // Set max velocity
    joints[i]->setAcceleration(joints_params[i]->max_accel);  // Set max acceleration
  }

  // Then give them to MultiStepper to manage
  // steppers.addStepper(joint1);
  // steppers.addStepper(joint2);
  // steppers.addStepper(joint3);
  // steppers.addStepper(joint4);
  // steppers.addStepper(joint5);

  // Configure gripper servo
  gripper.attach(11);
  
  digitalWrite(13, 1); // Toggle LED to indicate setup is complete
}


void loop() 
{
  if (Serial.available() > 0) 
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("Joint_")) 
    {
      int joint_num;
      long position, velocity;

      // Check for position and velocity control
      if (sscanf(command.c_str(), "Joint_%d position %ld velocity %ld", &joint_num, &position, &velocity) == 3 && joint_num >= 1 && joint_num <= NUM_JOINTS) 
      {
        int joint_index = joint_num - 1;
        position = constrain(position, joints_params[joint_index]->min_travel, joints_params[joint_index]->max_travel);
        velocity = constrain(velocity, (long)joints_params[joint_index]->min_vel, (long)joints_params[joint_index]->max_vel);

        joints[joint_index]->setMaxSpeed(velocity);
        joints[joint_index]->moveTo(position);
        is_position_control = true;

        Serial.print("Command received for Joint_");
        Serial.print(joint_num);
        Serial.print(" position set to ");
        Serial.print(position);
        Serial.print(" and velocity set to ");
        Serial.println(velocity);
      }
      // Check for position control only
      else if (sscanf(command.c_str(), "Joint_%d position %ld", &joint_num, &position) == 2 && joint_num >= 1 && joint_num <= NUM_JOINTS) 
      {
        int joint_index = joint_num - 1;
        position = constrain(position, joints_params[joint_index]->min_travel, joints_params[joint_index]->max_travel);

        // Need to add default velocity here - in case the previous setMaxSpeed not in the desired value
        joints[joint_index]->setMaxSpeed(joints_params[joint_index]->default_vel);
        joints[joint_index]->moveTo(position);
        is_position_control = true;

        Serial.print("Command received for Joint_");
        Serial.print(joint_num);
        Serial.print(" position set to ");
        Serial.println(position);
      }
      // Check for velocity control only
      else if (sscanf(command.c_str(), "Joint_%d velocity %ld", &joint_num, &velocity) == 2 && joint_num >= 1 && joint_num <= NUM_JOINTS) 
      {
        int joint_index = joint_num - 1;
        if (velocity > (long)joints_params[joint_index]->max_vel)
        {
          velocity = (long)joints_params[joint_index]->max_vel;
        }
        else if (velocity < (long)joints_params[joint_index]->min_vel)
        {
          velocity < (long)joints_params[joint_index]->min_vel;
        }

        joints[joint_index]->setSpeed(velocity);
        is_position_control = false;

        Serial.print("Command received for Joint_");
        Serial.print(joint_num);
        Serial.print(" velocity set to ");
        Serial.println(velocity);
      }
      else 
      {
        Serial.println("Unknown joint command format received");
      }
    }
    else if (command.startsWith("READ_STATE")) 
    {
      int joint_num;
      if (sscanf(command.c_str(), "READ_STATE Joint_%d", &joint_num) == 1 && joint_num >= 1 && joint_num <= NUM_JOINTS) 
      {
        int joint_index = joint_num - 1;
        long current_position = joints[joint_index]->currentPosition();
        long current_velocity = joints[joint_index]->speed();

        if (current_position == joint_positions[joint_index])
          current_velocity = 0;

        Serial.print("Joint_");
        Serial.print(joint_num);
        Serial.print(" curr_pos ");
        Serial.print(current_position);
        Serial.print(" curr_vel ");
        Serial.println(current_velocity);

        joint_positions[joint_index] = current_position;
        joint_velocities[joint_index] = current_velocity;
      } 
      else 
      {
        Serial.println("Unknown joint requested");
      }
    }
    else if (command.startsWith("GRIP ")) 
    {
      int angle;
      if (sscanf(command.c_str(), "GRIP %d", &angle) == 1) 
      {
        gripper.write(angle);
        Serial.print("Gripper angle set to ");
        Serial.println(angle);
      }
    } 
    else 
    {
      Serial.println("Unknown command received");
    }
  }

  // Run steppers based on the global control mode flag
  for (int i = 0; i < NUM_JOINTS; ++i) 
  {
    if (is_position_control) 
    {
      joints[i]->run();
    } 
    else // velocity control
    {
      joints[i]->runSpeed();
    }
  }
}

