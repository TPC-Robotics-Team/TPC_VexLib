#pragma once
#include "pros/motor_group.hpp"

class DifferentialDrive
{
  private:
    pros::MotorGroup& leftMotors;
    pros::MotorGroup& rightMotors;

  public:
    DifferentialDrive(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors);

    void tank(bool useHeadingHold = false);
    void arcade(bool useHeadingHold = false);
    void curvature(bool useHeadingHold = false);
};

class Intake
{
  private:
    pros::MotorGroup& IntakeMotors;

  public:
    Intake(pros::MotorGroup& IntakeMotors);

    void intake_control(int voltage = 12000);
};