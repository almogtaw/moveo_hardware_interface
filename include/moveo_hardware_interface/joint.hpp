#ifndef JOINT_HPP
#define JOINT_HPP

#include <string>

class Joint {
public:
    // Attributes
    std::string name;
    int steps;
    double pos;  // Position in radians
    double vel;  // Velocity in radians/sec
    float steps_per_rad;
    bool is_position_control;

    // Constructor
    Joint(const std::string& joint_name, float steps_per_rad = 1500.0)
        : name(joint_name), steps(0), pos(0.0), vel(0.0), steps_per_rad(steps_per_rad), is_position_control(true) {}

    // Method to convert steps to radians
    double stepsToRadians() const {
        return steps / steps_per_rad;
    }

    // Method to set position from steps
    void setPositionFromSteps(int step_count) {
        steps = step_count;
        pos = stepsToRadians();  // Update position in radians
    }

    // Method to set velocity in steps and convert to radians/sec
    void setVelocityFromSteps(int step_velocity) {
        vel = step_velocity / steps_per_rad;  // Update velocity in radians/sec
    }

    // Convert radians to steps
    int radiansToSteps(double radians) const
    {
        return static_cast<int>(radians * steps_per_rad);
    }
};

#endif // JOINT_HPP
