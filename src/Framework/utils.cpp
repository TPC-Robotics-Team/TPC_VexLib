#include "utils.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// Utils & Helper functions

// Joystick Deadband
// Makes inputs near zero turn into zero to eliminate small movements
double deadband(double value, double threshold){
    if (std::abs(value) < threshold) return 0;
    // Linear interpolation to ensure continuity
    double sign = (value > 0) ? 1.0 : -1.0;
    double t = (std::abs(value) - threshold) / (MAX_JOYSTICK - threshold);
    return sign * std::lerp(0.0, MAX_JOYSTICK, t);
}


// Cubic Input Shaping
// Shapes inputs to add weight to high speeds and control in lower speeds
double shapeInput(double input, double a){
    double x = input / MAX_JOYSTICK;
    double shaped = a * x * x * x + (1 - a) * x;
    return shaped * 127;
}



// Wrap Angle
// Wraps angles from -180 to 180
double wrapAngle(double x){
    x = fmod (x + 180.0, 360.0);
    if (x < 0){
        x += 360.0;
    }
    return x - 180.0;
}


// Angle Error
// Returns the shortest angle error for the robot/a mechanism to turn
double angleError(double target, double current){
    return wrapAngle(target - current);
}


// Normalize Drive For Two-motor Arcade Drive
// Maps [-254, 254] from kinematics to [-127, 127] for motors using scaling (maintains curvature)
void normalizeDriveTwoMotors(double &left, double &right){
    double maxMagnitude = fmax(std::abs(left), std::abs(right));

    if (maxMagnitude > MAX_JOYSTICK){
        left = left * MAX_JOYSTICK / maxMagnitude;
        right = right * MAX_JOYSTICK / maxMagnitude;
    }
}

// Joystick to Voltage Converting
int joystickToVoltage(int value){
    return value * 12000.0 / MAX_JOYSTICK;
}

// IMU Initialisation
void imuInit(pros::Imu &IMU){
    IMU.reset();

    while(IMU.is_calibrating()){
        pros::delay(20);
    }
}












