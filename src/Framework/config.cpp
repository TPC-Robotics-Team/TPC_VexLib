#include "custom/config.hpp"
#include "custom/subsystems.hpp"

int IMU_PORT;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors({1});
pros::MotorGroup RightMotors({-2});

pros::MotorGroup IntakeMotors({3, 4});

pros::Motor PivotMotor(3);
pros::Motor ClawMotor(4);

pros::Imu driveIMU(IMU_PORT);

DifferentialDrive drive(LeftMotors, RightMotors, driveIMU);
Intake intake(IntakeMotors);
OneDOFArm arm(PivotMotor, ClawMotor);

