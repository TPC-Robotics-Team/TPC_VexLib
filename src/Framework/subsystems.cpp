#include "custom/subsystems.hpp"
#include "custom/const.hpp"
#include "custom/utils.hpp"
#include <cmath>

// Differential Drive Class
DifferentialDrive::DifferentialDrive(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors)
    : leftMotors(leftMotors), rightMotors(rightMotors)
{
}

// Tank Drive
// One joystick controls an entire side of movement
void DifferentialDrive::tank(int left, int right)
{
    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}

// Arcade Drive
// One joystick controls throttle, and the other controls steer
void DifferentialDrive::arcade(int throttle, int steer)
{
    int left, right;

    left = throttle + steer;
    right = throttle - steer;

    double maxMagnitude = fmax(std::abs(left), std::abs(right));

    if (maxMagnitude > MAX_JOYSTICK)
    {
        left = left * MAX_JOYSTICK / maxMagnitude;
        right = right * MAX_JOYSTICK / maxMagnitude;
    }

    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}

// Curvature Drive
// Arcade Drive, but with a curvature term to make steering smooth
void DifferentialDrive::curvature(int throttle, int steer, bool quickTurn, double correction)
{
    double left, right;

    if (quickTurn || throttle == 0)
    {
        left = steer;
        right = -steer;
    }

    else
    {
        double steerNorm = (double)steer / MAX_JOYSTICK;
        left = throttle * (1 + steerNorm);
        right = throttle * (1 - steerNorm);
    }

    double maxMagnitude = fmax(std::abs(left), std::abs(right));

    if (maxMagnitude > MAX_JOYSTICK)
    {
        left = left * MAX_JOYSTICK / maxMagnitude;
        right = right * MAX_JOYSTICK / maxMagnitude;
    }

    left += correction;
    right -= correction;

    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}
//Intake//
void Intake::set(int power)
{
    leftintake = power;
    rightintake = power;
}
void Intake::intake_control()
{
    if (pros::controller_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        set(127);
    }
    else if (pros::controller_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        set(-127);
    }
    else
    {
        set(0);
    }
}
