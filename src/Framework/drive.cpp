#include "utils.hpp"
#include "drive.hpp"
#include <cmath>

constexpr double WHEEL_DIAMETER = 10.16; // Centimetres
constexpr double TICKS_PER_REV = 900; // Encoder constants for VEX Motor

// Differential Drive Class
DifferentialDrive::DifferentialDrive(
    pros::MotorGroup &LeftMotors,
    pros::MotorGroup &RightMotors)
    : LeftMotors(LeftMotors)
    , RightMotors(RightMotors)
{}


// Tank Drive
// One joystick controls an entire side of movement
void DifferentialDrive::tank(int left, int right){
    LeftMotors.move_voltage(joystickToVoltage(left));
    RightMotors.move_voltage(joystickToVoltage(right));
}


// Arcade Drive
// One joystick controls throttle, and the other controls steer
void DifferentialDrive::arcade(int throttle, int steer){
    int left, right;

    left = throttle + steer;
    right = throttle - steer;

    double maxMagnitude = fmax(std::abs(left), std::abs(right));

    if (maxMagnitude > MAX_JOYSTICK){
        left = left * MAX_JOYSTICK / maxMagnitude;
        right = right * MAX_JOYSTICK / maxMagnitude;
    }

    LeftMotors.move_voltage(joystickToVoltage(left));
    RightMotors.move_voltage(joystickToVoltage(right));
}


// Curvature Drive
// Arcade Drive, but with a curvature term to make steering smooth
void DifferentialDrive::curvature(int throttle, int steer, bool quickTurn, double correction){
    double left, right;
    
    if (quickTurn || throttle == 0){
        left = steer;
        right = -steer;
    }

    else{
        double steerNorm = (double)steer / MAX_JOYSTICK;
        left = throttle * (1 + steerNorm);
        right = throttle * (1 - steerNorm);
    }

    double maxMagnitude = fmax(std::abs(left), std::abs(right));

    if (maxMagnitude > MAX_JOYSTICK){
            left = left * MAX_JOYSTICK / maxMagnitude;
            right = right * MAX_JOYSTICK / maxMagnitude;
        }

    left += correction;
    right -= correction;

    LeftMotors.move_voltage(joystickToVoltage(left));
    RightMotors.move_voltage(joystickToVoltage(right));
}


// Centimetres to Encoder Tick Conversion
double cmToTick(double cm){
    return (cm / (M_PI * WHEEL_DIAMETER)) * TICKS_PER_REV;
}

