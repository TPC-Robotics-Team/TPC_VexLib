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
Intake::Intake(pros::MotorGroup& IntakeMotors) : IntakeMotors(IntakeMotors)
{
}
void Intake::set(int power)
{
    IntakeMotors.move_voltage(joystickToVoltage(power));
}
void Intake::intake_control()
{
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
        set(127); //*value depends*//
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) 
    {
        set(-127);//*value depends*//
    }
    else
    {
        set(0);
    }
}
