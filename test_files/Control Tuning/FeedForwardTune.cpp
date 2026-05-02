#include "main.h"
#include "pros/misc.h"
#include "custom/auton.hpp"



pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors ({1});
pros::MotorGroup RightMotors ({-2});


AutonDifferentialDrive auton_drive(LeftMotors, RightMotors);


void autonomous() {
    int TEST_VOLTAGE;

    auton_drive.tank(TEST_VOLTAGE, TEST_VOLTAGE);
}

void opcontrol() {
    while (true){
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            autonomous();
        };
    }
}