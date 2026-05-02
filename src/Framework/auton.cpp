#include "custom/auton.hpp"

// Autonomous control for a differential drive robot. This class provides
// methods to control the left and right motors of the robot independently,
// allowing for precise movement during autonomous routines.

AutonDifferentialDrive::AutonDifferentialDrive(pros::MotorGroup& LeftMotors, pros::MotorGroup& RightMotors)
    : LeftMotors(LeftMotors), RightMotors(RightMotors)
{
}

void AutonDifferentialDrive::tank(int left, int right)
{
    LeftMotors.move_voltage(left);
    RightMotors.move_voltage(right);
}