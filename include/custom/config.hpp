#pragma once
#include "pros/motor_group.hpp"
#include "custom/subsystems.hpp"
#include "pros/imu.hpp"

extern pros::Controller master;
extern pros::MotorGroup LeftMotors;
extern pros::MotorGroup RightMotors;
extern pros::Imu driveIMU;

extern pros::MotorGroup Intake;
extern DifferentialDrive drive;


