#include "main.h"
#include "custom/control.hpp"
#include "pros/misc.h"
#include "custom/subsystems.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors({1});

pros::MotorGroup RightMotors({-2});

pros::MotorGroup Intake({4, -11});

DifferentialDrive drive(LeftMotors, RightMotors);

SlewLimiter slewlimit(5, 7);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}