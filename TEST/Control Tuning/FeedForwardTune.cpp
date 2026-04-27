#include "main.h"
#include "pros/misc.h"
#include "auton.hpp"
#include <cstdio>

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup LeftMotors ({1});
pros::MotorGroup RightMotors ({-2});

AutonDifferentialDrive auton_drive(LeftMotors, RightMotors);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    int milivolt = 0;
    
}

void opcontrol() {
    while (true){
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            autonomous();
        };
        pros::delay(20);
    }
}