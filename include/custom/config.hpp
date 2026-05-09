#pragma once
#include "pros/motor_group.hpp"
#include "custom/control.hpp"
#include "custom/subsystems.hpp"
#include "custom/assistive_teleop.hpp"
#include "pros/imu.hpp"

extern pros::Controller master;
extern pros::MotorGroup LeftMotors;
extern pros::MotorGroup RightMotors;
extern pros::Imu IMU;

extern pros::MotorGroup Intake;
extern DifferentialDrive drive;

extern SlewLimiter throttlelimit;
extern SlewLimiter steerlimit;
extern PID headingPID;
extern HeadingHold headinghold;
