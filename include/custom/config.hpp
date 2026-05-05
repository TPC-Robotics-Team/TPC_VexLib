#pragma once
#include "pros/motor_group.hpp"
#include "custom/control.hpp"
#include "custom/subsystems.hpp"

extern pros::Controller master;

extern pros::MotorGroup LeftMotors;

extern pros::MotorGroup RightMotors;

extern pros::MotorGroup Intake;

extern DifferentialDrive drive;

extern SlewLimiter slewlimit;