#include "custom/config.hpp"
#include "custom/assistive_teleop.hpp"

int IMU_PORT;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors({1});
pros::MotorGroup RightMotors({-2});
pros::MotorGroup Intake({});
pros::Imu IMU(IMU_PORT);

DifferentialDrive drive(LeftMotors, RightMotors);

SlewLimiter throttlelimit(5, 10);
SlewLimiter steerlimit(5, 10);

PID headingPID(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
HeadingHold headinghold(IMU, headingPID);
