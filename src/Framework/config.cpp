#include "custom/config.hpp"

int IMU_PORT;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors({1});
pros::MotorGroup RightMotors({-2});
pros::MotorGroup Intake({});
pros::Imu driveIMU(IMU_PORT);

DifferentialDrive drive(LeftMotors, RightMotors, driveIMU);


