#include "auton.hpp"


AutonDifferentialDrive::AutonDifferentialDrive(
    pros::MotorGroup &LeftMotors,
    pros::MotorGroup &RightMotors)
    : LeftMotors(LeftMotors)
    , RightMotors(RightMotors)
{}

void AutonDifferentialDrive::tank(int left, int right){
    LeftMotors.move_voltage(left);
    RightMotors.move_voltage(right);
}