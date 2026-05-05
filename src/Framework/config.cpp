#include "custom/config.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors({});  
pros::MotorGroup RightMotors({});
pros::MotorGroup Intake({});

DifferentialDrive drive(LeftMotors, RightMotors);

SlewLimiter slewlimit(5, 7);