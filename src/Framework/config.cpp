#include "custom/config.hpp"
#include "custom/assistive_teleop.hpp"

int IMU_PORT;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors({});
pros::MotorGroup RightMotors({});
pros::MotorGroup Intake({});
pros::Imu IMU(IMU_PORT);

DifferentialDrive drive(LeftMotors, RightMotors);

SlewLimiter slewlimit(5, 7);

PID headingPID(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
HeadingHold headinghold(IMU, headingPID);
