#pragma once
#include "pros/motor_group.hpp"

class DifferentialDrive
{
  private:
    pros::MotorGroup& LeftMotors;
    pros::MotorGroup& RightMotors;

  public:
    DifferentialDrive(pros::MotorGroup& LeftMotors, pros::MotorGroup& RightMotors);

    void tank(int left, int right);
    void arcade(int throttle, int steer);
    void curvature(int throttle, int steer, bool quickTurn, double correction = 0);
};
class Intake
{
  private:
    pros::MotorGroup& IntakeMotors;

  public:
    Intake(pros::MotorGroup& IntakeMotors);

    void set(int power);
    void intake_control();
};