#include "main.h"
#include "custom/config.hpp"

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