#include "main.h"
#include "pros/misc.h"
#include "drive.hpp"



pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors ({1});

pros::MotorGroup RightMotors ({-2});

DifferentialDrive drive(LeftMotors, RightMotors);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true){
        int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        bool quickTurn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        drive.curvature(throttle, steer, quickTurn);

       pros::delay(20);
    }
}