#include "custom/subsystems.hpp"
#include "custom/const.hpp"
#include "custom/utils.hpp"
#include "custom/config.hpp"
#include "custom/assistive_teleop.hpp"
#include <cmath>

/*

Differential Drive Class

*/

DifferentialDrive::DifferentialDrive(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors)
    : leftMotors(leftMotors), rightMotors(rightMotors)
{
}

// Tank Drive

void DifferentialDrive::tank(bool useHeadingHold)
{
    int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    left = shapeInput(left);
    right = shapeInput(right);

    left = throttlelimit.update(left);
    right = steerlimit.update(right);

    if (useHeadingHold)
    {
        double throttle = (left + right) / 2.0;
        double steer = (right - left) / 2.0;

        double correction = headinghold.update(throttle, steer);

        left = std::clamp(left + correction, -127.0, 127.0);
        right = std::clamp(right - correction, -127.0, 127.0);
    }

    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}

// Arcade Drive

void DifferentialDrive::arcade(bool useHeadingHold)
{
    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int steer = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    throttle = shapeInput(throttle);
    steer = shapeInput(steer);

    throttle = throttlelimit.update(throttle);
    steer = steerlimit.update(steer);

    int left = throttle + steer;
    int right = throttle - steer;

    double maxMagnitude = fmax(std::abs(left), std::abs(right));

    if (maxMagnitude > MAX_JOYSTICK)
    {
        left = left * MAX_JOYSTICK / maxMagnitude;
        right = right * MAX_JOYSTICK / maxMagnitude;
    }

    if (useHeadingHold)
    {
        double correction = headinghold.update(throttle, steer);

        left = std::clamp(left + correction, -127.0, 127.0);
        right = std::clamp(right - correction, -127.0, 127.0);
    }

    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}

// Curvature Drive

void DifferentialDrive::curvature(bool useHeadingHold)
{
    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int steer = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    bool quickTurn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    throttle = shapeInput(throttle);
    steer = shapeInput(steer);

    throttle = throttlelimit.update(throttle);
    steer = steerlimit.update(steer);

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

    if (useHeadingHold && !quickTurn && throttle != 0)
    {
        double correction = headinghold.update(throttle, steer);

        left = std::clamp(left + correction, -127.0, 127.0);
        right = std::clamp(right - correction, -127.0, 127.0);
    }

    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}

/*

Intake Class

*/

Intake::Intake(pros::MotorGroup& IntakeMotors) : IntakeMotors(IntakeMotors) {}

// Intake Control

void Intake::intake_control(int voltage)
{
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
        IntakeMotors.move_voltage(voltage);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        IntakeMotors.move_voltage(-voltage);
    }
    else
    {
        IntakeMotors.move_voltage(0);
    }
}
