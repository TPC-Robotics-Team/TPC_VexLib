#pragma once
#include "pros/motor_group.hpp"
#include "custom/const.hpp"

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

    void intake_control(int voltage = MAX_MILIVOLTS);
};

class OneDOFClaw
{
  private:
    pros::MotorGroup& m_pivot;
    pros::Motor& m_claw;
    double m_gear_ratio;

  public:
    OneDOFClaw(pros::MotorGroup& pivot, pros::Motor& claw, double m_gear_ratio = 1);
   
};