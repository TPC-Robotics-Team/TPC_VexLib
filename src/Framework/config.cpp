#include "custom/config.hpp"
#include "custom/subsystems.hpp"

int IMU_PORT = 11;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors({5});
pros::MotorGroup RightMotors({-4});

pros::Motor PivotMotor(-8);
pros::Motor ClawMotor(9);

pros::Imu driveIMU(IMU_PORT);

DifferentialDrive drive(LeftMotors, RightMotors, driveIMU);
OneDOFArm arm(PivotMotor, ClawMotor);

